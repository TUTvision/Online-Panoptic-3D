/* raiviol 5/2021
*/

#include <simple_integrator.h>

namespace fusion
{

void SimpleTsdfIntegrator::integratePointCloud(
    const voxblox::Transformation& T_G_C,
    const voxblox::Pointcloud& points_C,
    const voxblox::Colors& colors,
    const bool freespace_points)
{
    voxblox::timing::Timer integrate_timer("integrate/simple");
    CHECK_EQ(points_C.size(), colors.size());

    std::unique_ptr<voxblox::ThreadSafeIndex> index_getter(
        voxblox::ThreadSafeIndexFactory::get(config_.integration_order_mode, points_C));

    std::list<std::thread> integration_threads;
    for (size_t i = 0; i < config_.integrator_threads; ++i)
    {
        integration_threads.emplace_back(&SimpleTsdfIntegrator::integrateFunction,
                                         this, T_G_C, points_C, colors,
                                         freespace_points, index_getter.get());
    }

    for (std::thread& thread : integration_threads)
    {
        thread.join();
    }

    update_voxels();

    integrate_timer.Stop();

    voxblox::timing::Timer insertion_timer("inserting_missed_blocks");
    updateLayerWithStoredBlocks();
    insertion_timer.Stop();

    send_confirmation_message(true);
}

void SimpleTsdfIntegrator::integrateFunction(
    const voxblox::Transformation& T_G_C,
    const voxblox::Pointcloud& points_C,
    const voxblox::Colors& colors,
    const bool freespace_points,
    voxblox::ThreadSafeIndex* index_getter)
{
    DCHECK(index_getter != nullptr);

    size_t point_idx;
    while (index_getter->getNextIndex(&point_idx))
    {
        const voxblox::Point& point_C = points_C[point_idx];
        const voxblox::Color& color = colors[point_idx];
        bool is_clearing;

        if (!isPointValid(point_C, freespace_points, &is_clearing))
        {
            continue;
        }

        double divisor = 255;

        // because of opencv, images are in bgr format
        const uint8_t local_id   = color.b;
        const uint8_t category   = color.g;
        const double confidence  = color.r / divisor;

        const voxblox::Point origin = T_G_C.getPosition();
        const voxblox::Point point_G = T_G_C * point_C;

        voxblox::RayCaster ray_caster(origin, point_G, is_clearing,
                                      config_.voxel_carving_enabled,
                                      config_.max_ray_length_m, voxel_size_inv_,
                                      config_.default_truncation_distance);

        voxblox::Block<voxblox::TsdfVoxel>::Ptr block = nullptr;
        voxblox::BlockIndex block_idx;
        voxblox::GlobalIndex global_voxel_idx;
        while (ray_caster.nextRayIndex(&global_voxel_idx))
        {
            voxblox::TsdfVoxel* voxel =
                allocateStorageAndGetVoxelPtr(global_voxel_idx, &block, &block_idx);

            const float weight = getVoxelWeight(point_C);

            std::pair<bool, float> res = updateTsdfVoxel(
                origin, point_G, global_voxel_idx, weight, voxel);

            bool is_occupied = res.first;
            bool weighting_val = res.second;

            if( is_occupied )
            {
                std::lock_guard<std::mutex> voxel_guard(add_voxel_mtx_);
                add_measurement(local_id, category, global_voxel_idx, weighting_val);
            }
        }
    }
}

} // end namespace fusion

/* raiviol 5/2021
*/

#include <fast_integrator.h>

namespace fusion
{

void FastTsdfIntegrator::integrateFunction(
    const voxblox::Transformation& T_G_C,
    const voxblox::Pointcloud& points_C,
    const voxblox::Colors& colors,
    const bool freespace_points,
    voxblox::ThreadSafeIndex* index_getter)
{
    DCHECK(index_getter != nullptr);
    size_t point_idx;

    while( index_getter->getNextIndex(&point_idx) &&
           (std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now() - integration_start_time_
            ).count() < config_.max_integration_time_s * 1000000) )
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
        const uint8_t local_id  = color.b;
        const uint8_t category  = color.g;
        const double confidence = color.r / divisor;

        const voxblox::Point origin  = T_G_C.getPosition();
        const voxblox::Point point_G = T_G_C * point_C;

        // Checks to see if another ray in this scan has already started 'close'
        // to this location. If it has then we skip ray casting this point. We
        // measure if a start location is 'close' to another points by inserting
        // the point into a set of voxels. This voxel set has a resolution
        // start_voxel_subsampling_factor times higher then the voxel size.
        voxblox::GlobalIndex global_voxel_idx =
            voxblox::getGridIndexFromPoint<voxblox::GlobalIndex>(
                point_G,
                config_.start_voxel_subsampling_factor * voxel_size_inv_
            );

        if (!start_voxel_approx_set_.replaceHash(global_voxel_idx))
        {
            continue;
        }

        constexpr bool cast_from_origin = false;
        voxblox::RayCaster ray_caster(
            origin, point_G, is_clearing,
            config_.voxel_carving_enabled,
            config_.max_ray_length_m,
            voxel_size_inv_,
            config_.default_truncation_distance,
            cast_from_origin);

        int64_t consecutive_ray_collisions = 0;

        voxblox::Block<voxblox::TsdfVoxel>::Ptr block = nullptr;
        voxblox::BlockIndex block_idx;
        while (ray_caster.nextRayIndex(&global_voxel_idx))
        {
            // Check if the current voxel has been seen by any ray cast this scan.
            // If it has increment the consecutive_ray_collisions counter, otherwise
            // reset it. If the counter reaches a threshold we stop casting as the
            // ray is deemed to be contributing too little new information.
            if (!voxel_observed_approx_set_.replaceHash(global_voxel_idx))
            {
                ++consecutive_ray_collisions;
            }
            else
            {
                consecutive_ray_collisions = 0;
            }
            if (consecutive_ray_collisions > config_.max_consecutive_ray_collisions)
            {
                break;
            }

            voxblox::TsdfVoxel* voxel =
                allocateStorageAndGetVoxelPtr(global_voxel_idx, &block, &block_idx);

            const float weight = getVoxelWeight(point_C);

            std::pair<bool, float> res = updateTsdfVoxel(
                origin, point_G, global_voxel_idx, weight, voxel);

            bool is_occupied = res.first;
            bool weighting_val = res.second;

            if( !is_clearing && is_occupied )
            {
                std::lock_guard<std::mutex> voxel_guard(add_voxel_mtx_);
                add_measurement(local_id, category, global_voxel_idx, weighting_val);
            }
        }
    }
}

void FastTsdfIntegrator::integratePointCloud(
    const voxblox::Transformation& T_G_C,
    const voxblox::Pointcloud& points_C,
    const voxblox::Colors& colors,
    const bool freespace_points)
{
    using std::chrono::high_resolution_clock;
    using std::chrono::duration;

    auto t1 = high_resolution_clock::now();

    voxblox::timing::Timer integrate_timer("integrate/fast");
    CHECK_EQ(points_C.size(), colors.size());

    integration_start_time_ = std::chrono::steady_clock::now();

    static int64_t reset_counter = 0;
    if( (++reset_counter) >= config_.clear_checks_every_n_frames )
    {
        reset_counter = 0;
        start_voxel_approx_set_.resetApproxSet();
        voxel_observed_approx_set_.resetApproxSet();
    }

    std::unique_ptr<voxblox::ThreadSafeIndex> index_getter(
        voxblox::ThreadSafeIndexFactory::get(config_.integration_order_mode, points_C));

    std::list<std::thread> integration_threads;
    for( size_t i = 0; i < config_.integrator_threads; ++i )
    {
        integration_threads.emplace_back(
            &FastTsdfIntegrator::integrateFunction,
            this, T_G_C, points_C, colors,
            freespace_points, index_getter.get());
    }

    for (std::thread& thread : integration_threads)
    {
        thread.join();
    }

    integrate_timer.Stop();

    update_voxels();

    voxblox::timing::Timer insertion_timer("inserting_missed_blocks");
    updateLayerWithStoredBlocks();
    insertion_timer.Stop();

    auto t2 = high_resolution_clock::now();

    duration<double> tdiff = t2 - t1;
    save_time(tdiff, fusion_timings_);

    send_confirmation_message(true);
}

} // namespace fusion

/* raiviol 5/2021
*/

#include <merged_integrator.h>

namespace fusion
{

void MergedTsdfIntegrator::integratePointCloud(
    const voxblox::Transformation& T_G_C,
    const voxblox::Pointcloud& points_C,
    const voxblox::Colors& colors,
    const bool freespace_points)
{
    using std::chrono::high_resolution_clock;
    using std::chrono::duration;

    auto t1 = high_resolution_clock::now();

    voxblox::timing::Timer integrate_timer("integrate/merged");
    CHECK_EQ(points_C.size(), colors.size());

    // Pre-compute a list of unique voxels to end on.
    // Create a hashmap: VOXEL INDEX -> index in original cloud.
    voxblox::LongIndexHashMapType<voxblox::AlignedVector<size_t>>::type voxel_map;
    // This is a hash map (same as above) to all the indices that need to be
    // cleared.
    voxblox::LongIndexHashMapType<voxblox::AlignedVector<size_t>>::type clear_map;

    std::unique_ptr<voxblox::ThreadSafeIndex> index_getter(
        voxblox::ThreadSafeIndexFactory::get(config_.integration_order_mode, points_C));

    bundleRays(T_G_C, points_C, freespace_points, index_getter.get(), &voxel_map,
               &clear_map);

    integrateRays(T_G_C, points_C, colors, config_.enable_anti_grazing, false,
                  voxel_map, clear_map);

    voxblox::timing::Timer clear_timer("integrate/clear");

    integrateRays(T_G_C, points_C, colors, config_.enable_anti_grazing, true,
                  voxel_map, clear_map);

    clear_timer.Stop();

    update_voxels();

    voxblox::timing::Timer insertion_timer("inserting_missed_blocks");
    updateLayerWithStoredBlocks();
    insertion_timer.Stop();

    integrate_timer.Stop();

    auto t2 = high_resolution_clock::now();

    duration<double> tdiff = t2 - t1;
    save_time(tdiff, fusion_timings_);

    send_confirmation_message(true);
}

void MergedTsdfIntegrator::bundleRays(
    const voxblox::Transformation& T_G_C,
    const voxblox::Pointcloud& points_C,
    const bool freespace_points,
    voxblox::ThreadSafeIndex* index_getter,
    voxblox::LongIndexHashMapType<voxblox::AlignedVector<size_t>>::type* voxel_map,
    voxblox::LongIndexHashMapType<voxblox::AlignedVector<size_t>>::type* clear_map)
{
    DCHECK(voxel_map != nullptr);
    DCHECK(clear_map != nullptr);

    size_t point_idx;
    while (index_getter->getNextIndex(&point_idx))
    {
        const voxblox::Point& point_C = points_C[point_idx];

        bool is_clearing;
        if (!isPointValid(point_C, freespace_points, &is_clearing))
        {
            continue;
        }

        const voxblox::Point point_G = T_G_C * point_C;

        voxblox::GlobalIndex voxel_index =
            voxblox::getGridIndexFromPoint<voxblox::GlobalIndex>(point_G, voxel_size_inv_);

        if (is_clearing)
        {
            (*clear_map)[voxel_index].push_back(point_idx);
        } else
        {
            (*voxel_map)[voxel_index].push_back(point_idx);
        }
    }
    VLOG(3) << "Went from " << points_C.size() << " points to "
          << voxel_map->size() << " raycasts  and " << clear_map->size()
          << " clear rays.";
}

void MergedTsdfIntegrator::integrateRays(
    const voxblox::Transformation& T_G_C,
    const voxblox::Pointcloud& points_C,
    const voxblox::Colors& colors,
    bool enable_anti_grazing,
    bool clearing_ray,
    const voxblox::LongIndexHashMapType<voxblox::AlignedVector<size_t>>::type& voxel_map,
    const voxblox::LongIndexHashMapType<voxblox::AlignedVector<size_t>>::type& clear_map)
{
    // if only 1 thread just do function call, otherwise spawn threads
    if (config_.integrator_threads == 1)
    {
        constexpr size_t thread_idx = 0;
        integrateVoxels(T_G_C, points_C, colors, enable_anti_grazing, clearing_ray,
                        voxel_map, clear_map, thread_idx);
    }
    else
    {
        std::list<std::thread> integration_threads;
        for (size_t i = 0; i < config_.integrator_threads; ++i)
        {
            integration_threads.emplace_back(
                &MergedTsdfIntegrator::integrateVoxels, this, T_G_C, points_C, colors,
                enable_anti_grazing, clearing_ray, voxel_map, clear_map, i);
        }
        for (std::thread& thread : integration_threads)
        {
            thread.join();
        }
    }

    voxblox::timing::Timer insertion_timer("inserting_missed_blocks");
    updateLayerWithStoredBlocks();
    insertion_timer.Stop();
}

void MergedTsdfIntegrator::integrateVoxels(
    const voxblox::Transformation& T_G_C,
    const voxblox::Pointcloud& points_C,
    const voxblox::Colors& colors,
    bool enable_anti_grazing,
    bool clearing_ray,
    const voxblox::LongIndexHashMapType<voxblox::AlignedVector<size_t>>::type& voxel_map,
    const voxblox::LongIndexHashMapType<voxblox::AlignedVector<size_t>>::type& clear_map,
    size_t thread_idx)
{
    voxblox::LongIndexHashMapType<voxblox::AlignedVector<size_t>>::type::const_iterator it;
    size_t map_size;
    if (clearing_ray)
    {
        it = clear_map.begin();
        map_size = clear_map.size();
    }
    else
    {
        it = voxel_map.begin();
        map_size = voxel_map.size();
    }
    for (size_t i = 0; i < map_size; ++i)
    {
        if( ((i + thread_idx + 1) % config_.integrator_threads) == 0 )
        {
          integrateVoxel(T_G_C, points_C, colors, enable_anti_grazing, clearing_ray,
                         *it, voxel_map);
        }
        ++it;
    }
}

void MergedTsdfIntegrator::integrateVoxel(
    const voxblox::Transformation& T_G_C,
    const voxblox::Pointcloud& points_C,
    const voxblox::Colors& colors,
    bool enable_anti_grazing,
    bool clearing_ray,
    const std::pair<voxblox::GlobalIndex, voxblox::AlignedVector<size_t>>& kv,
    const voxblox::LongIndexHashMapType<voxblox::AlignedVector<size_t>>::type& voxel_map)
{
    if (kv.second.empty()) {return;}

    const voxblox::Point& origin = T_G_C.getPosition();
    voxblox::Point merged_point_C = voxblox::Point::Zero();
    voxblox::FloatingPoint merged_weight = 0.0;

    double divisor = 255;

    IdMap local_ids;
    unsigned int top_id = VOID_CLASS_ID;
    unsigned int top_n = 0;

    for (const size_t pt_idx : kv.second)
    {
        const voxblox::Point& point_C = points_C[pt_idx];
        const voxblox::Color& color = colors[pt_idx];

        // because of opencv, images are in bgr format
        const unsigned int local_id  = color.b;
        const unsigned int category  = color.g;
        const double confidence = color.r / divisor;

        if( category < N_CLASSES )
        {
            if( local_ids.count(local_id) == 0 )
            {
                local_ids[local_id] = std::make_shared<Id_struct>();
            }

            local_ids[local_id]->update_confidence(category);

            if( local_ids[local_id]->n > top_n )
            {
                top_n = local_ids[local_id]->n;
                top_id = local_id;
            }
        }
        const float point_weight = getVoxelWeight(point_C);
        if (point_weight < voxblox::kEpsilon) {continue;}

        merged_point_C = (merged_point_C * merged_weight + point_C * point_weight) /
                         (merged_weight + point_weight);
        merged_weight += point_weight;

        // only take first point when clearing
        if (clearing_ray) {break;}
    }

    const voxblox::Point merged_point_G = T_G_C * merged_point_C;

    voxblox::RayCaster ray_caster(
        origin, merged_point_G, clearing_ray,
        config_.voxel_carving_enabled, config_.max_ray_length_m,
        voxel_size_inv_, config_.default_truncation_distance);

    voxblox::GlobalIndex global_voxel_idx;
    while (ray_caster.nextRayIndex(&global_voxel_idx))
    {
        if (enable_anti_grazing)
        {
            // Check if this one is already the the block hash map for this
            // insertion. Skip this to avoid grazing.
            if( (clearing_ray || global_voxel_idx != kv.first) &&
                voxel_map.find(global_voxel_idx) != voxel_map.end() )
            {
                continue;
            }
        }
        voxblox::Block<voxblox::TsdfVoxel>::Ptr block = nullptr;
        voxblox::BlockIndex block_idx;
        voxblox::TsdfVoxel* voxel =
            allocateStorageAndGetVoxelPtr(global_voxel_idx, &block, &block_idx);

        std::pair<bool, float> res = updateTsdfVoxel(
            origin, merged_point_G, global_voxel_idx, merged_weight, voxel);

        bool is_occupied = res.first;
        bool weighting_val = res.second;

        if( !clearing_ray && is_occupied ) //&& top_id != VOID_CLASS_ID )
        {
            unsigned int category =
                (local_ids.count(top_id) > 0) ?
                    local_ids[top_id]->get_category(confidence_threshold_)
                    : VOID_CLASS_ID;

            add_measurement(top_id, category, global_voxel_idx, weighting_val);
        }
    }
}

} // namespace fusion

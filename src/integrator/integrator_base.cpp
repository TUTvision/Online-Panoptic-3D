/* raiviol 5/2021
*/

#include <integrator_base.h>

namespace fusion
{
TsdfIntegratorBase::TsdfIntegratorBase(
    const voxblox::TsdfIntegratorBase::Config& config,
    voxblox::Layer<voxblox::TsdfVoxel>* layer,
    fusion::Config& fusion_config,
    ros::Publisher confirm_pub)
        : voxblox::TsdfIntegratorBase(config, layer),
          tracker_(PanopticTracker(fusion_config)),
          confirm_pub_(confirm_pub),
          voxel_weighting_(fusion_config.weighting_strategy),
          confidence_threshold_(fusion_config.confidence_threshold),
          save_timings_(fusion_config.save_timings)
{
}

bool TsdfIntegratorBase::saveSegmentationCallback()
{
    double fusion_average_time = -1;
    if( save_timings_ )
    {
        fusion_average_time = get_avg_time(fusion_timings_);
    }
    return tracker_.save_segmentation(fusion_average_time);
}

bool TsdfIntegratorBase::createSegmentationCallback(
    std_srvs::Empty::Request&,
    std_srvs::Empty::Response&)
{
    return saveSegmentationCallback();
}

void TsdfIntegratorBase::send_confirmation_message(bool status)
{
    std_msgs::Bool msg;
    msg.data = status;

    confirm_pub_.publish(msg);
}

void TsdfIntegratorBase::add_measurement(
    const unsigned int id,
    const unsigned int category,
    const voxblox::GlobalIndex& global_voxel_idx,
    const double weighting_val)
{
    if( id != VOID_CLASS_ID && category != VOID_CLASS_ID )
    {
        std::lock_guard<std::mutex> voxel_guard(add_voxel_mtx_);
        tracker_.add_measurement(id, category, global_voxel_idx, weighting_val);
    }
    updated_voxels_.insert(global_voxel_idx);
}

void TsdfIntegratorBase::update_voxels_callback(
    VoxelSet::iterator first,
    VoxelSet::iterator last)
{
    while( first != last )
    {
        voxblox::Block<voxblox::TsdfVoxel>::Ptr block = nullptr;
        voxblox::BlockIndex block_idx;
        voxblox::TsdfVoxel* voxel =
            allocateStorageAndGetVoxelPtr(*first, &block, &block_idx);

        if(tracker_.map->count(*first) > 0)
        {
            unsigned int target_id = (*tracker_.map)[*first].first;
            voxel->color = (*tracker_.ids)[target_id]->color;
        }
        else
        {
            voxel->color = VOID_COLOR;
        }

        ++first;
    }
}

void TsdfIntegratorBase::update_voxels()
{
    {
        std::lock_guard<std::mutex> voxel_guard(tracker_update_mtx_);
        tracker_.update_map();
    }

    unsigned int n_voxels = updated_voxels_.size();

    if( n_voxels == 0 ){return;}

    unsigned int partition = n_voxels / n_threads_;

    std::vector<std::thread> threads;
    threads.reserve(n_threads_);

    VoxelSet::iterator first = updated_voxels_.begin();
    VoxelSet::iterator last = updated_voxels_.begin();

    unsigned int index = 0;

    for( unsigned int i = 0 ; i < n_threads_ ; ++i )
    {
        if( index + partition < n_voxels && i < n_threads_-1 )
        {
            std::advance(last, partition);
            index += partition;
        }
        else
        {
            last = updated_voxels_.end();
        }

        threads.push_back(
            std::thread(
                &TsdfIntegratorBase::update_voxels_callback, this,
                first, last
            )
        );

        first = last;
    }

    for( std::thread& t : threads )
    {
        t.join();
    }

    updated_voxels_.clear();
}

std::pair<bool, float> TsdfIntegratorBase::updateTsdfVoxel(
    const voxblox::Point& origin,
    const voxblox::Point& point_G,
    const voxblox::GlobalIndex& global_voxel_idx,
    const float weight,
    voxblox::TsdfVoxel* tsdf_voxel)
{
    DCHECK(tsdf_voxel != nullptr);

    const voxblox::Point voxel_center =
        voxblox::getCenterPointFromGridIndex(global_voxel_idx, voxel_size_);

    const float sdf = computeDistance(origin, point_G, voxel_center);

    float updated_weight = weight;
    // Compute updated weight in case we use weight dropoff. It's easier here
    // that in getVoxelWeight as here we have the actual SDF for the voxel
    // already computed.
    const voxblox::FloatingPoint dropoff_epsilon = voxel_size_;
    if (config_.use_weight_dropoff && sdf < -dropoff_epsilon)
    {
        updated_weight = weight * (config_.default_truncation_distance + sdf) /
                         (config_.default_truncation_distance - dropoff_epsilon);
        updated_weight = std::max(updated_weight, 0.0f);
    }

    // Compute the updated weight in case we compensate for sparsity. By
    // multiplicating the weight of occupied areas (|sdf| < truncation distance)
    // by a factor, we prevent to easily fade out these areas with the free
    // space parts of other rays which pass through the corresponding voxels.
    // This can be useful for creating a TSDF map from sparse sensor data (e.g.
    // visual features from a SLAM system). By default, this option is disabled.
    if (config_.use_sparsity_compensation_factor)
    {
        if (std::abs(sdf) < config_.default_truncation_distance)
        {
            updated_weight *= config_.sparsity_compensation_factor;
        }
    }

    // Lookup the mutex that is responsible for this voxel and lock it
    std::lock_guard<std::mutex> lock(mutexes_.get(global_voxel_idx));

    const float new_weight = tsdf_voxel->weight + updated_weight;

    float weight_val = 0;

    // it is possible to have weights very close to zero, due to the limited
    // precision of floating points dividing by this small value can cause nans
    if (new_weight < voxblox::kFloatEpsilon)
    {
        auto output = std::make_pair(false, weight_val);
        return output;
    }

    const float new_sdf =
        (sdf * updated_weight + tsdf_voxel->distance * tsdf_voxel->weight) /
        new_weight;

    tsdf_voxel->distance =
        (new_sdf > 0.0) ? std::min(config_.default_truncation_distance, new_sdf)
                        : std::max(-config_.default_truncation_distance, new_sdf);
    tsdf_voxel->weight = std::min(config_.max_weight, new_weight);

    bool is_occupied = false;
    if (std::abs(sdf) < config_.default_truncation_distance)
    {
        is_occupied = true;
    }

    if( voxel_weighting_ == "constant" )
    {
        weight_val = 1;
    }
    else
    {
        weight_val = (voxel_weighting_ == "distance") ? new_sdf : new_weight;
    }

    return std::make_pair(is_occupied, weight_val);
}

void TsdfIntegratorBase::save_time(std::chrono::duration<double> tdiff, std::vector<double>& vec)
{
    if( save_timings_ )
    {
        vec.push_back(tdiff.count());
    }
}

double TsdfIntegratorBase::get_avg_time(std::vector<double>& times)
{
    double sum = 0;
    for(auto t : times)
    {
        sum += t;
    }
    return sum / times.size();
}

} // end namespace fusion

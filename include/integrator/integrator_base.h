/* raiviol 5/2021
*/

#ifndef FUSION_INTEGRATOR_BASE_H
#define FUSION_INTEGRATOR_BASE_H

#include <panoptic_tracker.h>
#include <config.h>

#include <voxblox/integrator/tsdf_integrator.h>

#include <mutex>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <std_srvs/Empty.h>

#include <chrono>

namespace fusion
{
class TsdfIntegratorBase : public voxblox::TsdfIntegratorBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TsdfIntegratorBase(
        const voxblox::TsdfIntegratorBase::Config& config,
        voxblox::Layer<voxblox::TsdfVoxel>* layer,
        fusion::Config& fusion_config,
        ros::Publisher confirm_pub
    );

    bool saveSegmentationCallback();
    bool createSegmentationCallback(
        std_srvs::Empty::Request&,
        std_srvs::Empty::Response&
    );

    void send_confirmation_message(bool status);

protected:
    unsigned int n_threads_ = 8;

    PanopticTracker tracker_;
    std::string voxel_weighting_;
    double confidence_threshold_;
    VoxelSet updated_voxels_;

    std::mutex tracker_update_mtx_;
    std::mutex add_voxel_mtx_;

    ros::Publisher confirm_pub_;

    void add_measurement(
        const unsigned int id,
        const unsigned int category,
        const voxblox::GlobalIndex& global_voxel_idx,
        const double weighting_val
    );

    void update_voxels_callback(
        VoxelSet::iterator first,
        VoxelSet::iterator last
    );

    void update_voxels();
    std::pair<bool, float> updateTsdfVoxel(
        const voxblox::Point& origin,
        const voxblox::Point& point_G,
        const voxblox::GlobalIndex& global_voxel_index,
        const float weight,
        voxblox::TsdfVoxel* tsdf_voxel
    );

    bool save_timings_;
    std::vector<double> fusion_timings_;

    void save_time(std::chrono::duration<double> tdiff, std::vector<double>& vec);
    double get_avg_time(std::vector<double>& times);
};

} // end namespace fusion
#endif // FUSION_INTEGRATOR_BASE_H

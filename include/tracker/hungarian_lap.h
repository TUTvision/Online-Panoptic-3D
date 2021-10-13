/* raiviol 8/2021
*/

#ifndef FUSION_HUNGARIAN_H
#define FUSION_HUNGARIAN_H

#include <object_tracker_base.h>

#include <voxblox/core/common.h>
#include <dlib/optimization/max_cost_assignment.h>
#include <ros/console.h>

#include <stdexcept>

namespace fusion
{

class H_LAP : public Object_Tracker_Base
{
public:
    H_LAP(
        unsigned int min_id = 0,
        bool use_local_reference = true,
        bool normalise_likelihoods = true,
        double new_target_likelihood_ = 0.1,
        std::string likelihood_metric = "iou",
        std::shared_ptr<VoxelMap> reference_map = nullptr
    );
    ~H_LAP(){}

    std::shared_ptr<AssociationVector> process_measurements(
        ObjectMap& measurements,
        std::shared_ptr<IdSet> local_reference = nullptr,
        std::shared_ptr<VoxelSet> local_voxels = nullptr
    ) override;

protected:
    void likelihood_matrix_callback(
        std::shared_ptr<Object_struct> measurement,
        std::shared_ptr<Object_struct> target,
        std::shared_ptr<VoxelSet> local_voxels,
        unsigned int measurement_index,
        unsigned int target_index,
        dlib::matrix<double>& likelihood_matrix
    );

    void update_map();
    void update_map(
        ObjectMap& measurements,
        std::shared_ptr<AssociationVector> assignments
    );

    bool normalise_likelihoods_;
    double new_target_likelihood_;

    std::string likelihood_metric_;

    ObjectMap map_;
    std::shared_ptr<VoxelMap> reference_map_;
};

} // namespace fusion

#endif // FUSION_HUNGARIAN_H

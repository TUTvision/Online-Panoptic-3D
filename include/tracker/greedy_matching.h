/* raiviol 8/2021
*/

#ifndef FUSION_GREEDY_H
#define FUSION_GREEDY_H

#include <object_tracker_base.h>

#include <voxblox/core/common.h>
#include <dlib/optimization/max_cost_assignment.h>
#include <ros/console.h>

#include <stdexcept>

namespace fusion
{

class Greedy_LAP : public Object_Tracker_Base
{
public:
    Greedy_LAP(
        unsigned int min_id = 0,
        bool use_local_reference = true,
        double association_threshold_ = 0.25,
        std::string likelihood_metric = "iou"
    );
    ~Greedy_LAP(){}

    std::shared_ptr<AssociationVector> process_measurements(
        ObjectMap& measurements,
        ObjectMap& local_targets,
        std::shared_ptr<VoxelSet> local_voxels
    ) override;

protected:

    void update_map();
    void update_map(
        ObjectMap& measurements,
        std::shared_ptr<AssociationVector> assignments
    );

    double association_threshold_;

    std::string likelihood_metric_;
};

} // namespace fusion

#endif // FUSION_GREEDY_H

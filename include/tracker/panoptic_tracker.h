/* raiviol 5/2021
*/

#ifndef FUSION_PANOPTIC_TRACKING_H
#define FUSION_PANOPTIC_TRACKING_H

#include <common.h>

#include <config.h>
#include <scannet_classes.h>
#include <data_structures.h>
#include <distributions.h>

#include <outlier_rejection.h>
#include <object_tracker_base.h>
#include <hungarian_lap.h>
#include <greedy_matching.h>

#include <voxblox/core/common.h>
#include <dlib/optimization/max_cost_assignment.h>
#include <ros/console.h>

#include <memory>
#include <random>
#include <iterator>
#include <algorithm>

#include <thread>
#include <chrono>
#include <fstream>
#include <cmath>

namespace fusion
{

class PanopticTracker
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PanopticTracker();
    PanopticTracker(const fusion::Config& fusion_config);
    ~PanopticTracker(){};

    std::shared_ptr<IdMap> ids;
    std::shared_ptr<VoxelMap> map;

    void add_measurement(
        unsigned int id,
        unsigned int category,
        Voxel point,
        double weight
    );

    void update_map();
    //unsigned int data_association(measurement_vector& M);

    bool save_segmentation(double fusion_av = -1);

protected:

    void process_stuff_measurement(
        const unsigned int id,
        const unsigned int category,
        const Voxel point,
        const double weight
    );

    void process_static_measurement(
        const unsigned int id,
        const unsigned int category,
        const Voxel point,
        const double weight
    );

    void process_dynamic_measurement(
        const unsigned int id,
        const unsigned int category,
        const Voxel point,
        const double weight
    );

    void outlier_rejection_callback(std::shared_ptr<Object_struct> measurement);
    void sampling_callback(std::shared_ptr<Object_struct> measurement);
    void fuse_callback(std::shared_ptr<Object_struct> measurement, unsigned int global_id);

    void fuse(
        Voxel point,
        unsigned int id,
        unsigned int category,
        double weight
    );

    void new_object(unsigned int id);

    void update_point(Voxel point, unsigned int id, double weight);

    void save_time(std::chrono::duration<double> tdiff, std::vector<double>& vec);
    double get_avg_time(std::vector<double>& times);

    ObjectMap measurements_;
    std::shared_ptr<VoxelSet> local_voxels_;

    std::shared_ptr<Object_Tracker_Base> tracker_;
    std::shared_ptr<Outlier_Rejection> outlier_rejection_;

    std::vector<double> tracking_times_;
    std::vector<double> preprocess_times_;
    std::vector<double> association_times_;

    fusion::Config config_;

}; // class PanopticTracker
} // namespace fusion
#endif // FUSION_PANOPTIC_TRACKING_H

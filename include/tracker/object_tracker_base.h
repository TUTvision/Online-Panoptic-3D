/* raiviol 8/2021
*/

#ifndef FUSION_OBJECT_TRACKER_BASE_H
#define FUSION_OBJECT_TRACKER_BASE_H

#include<common.h>

#include<distributions.h>
#include<data_structures.h>

#include<vector>
#include<memory>

namespace fusion
{

class Object_Tracker_Base
{
public:
    Object_Tracker_Base(
        unsigned int min_id = 0,
        bool use_local_reference = true
    );
    virtual ~Object_Tracker_Base() = default;

    virtual std::shared_ptr<AssociationVector> process_measurements(
        ObjectMap& measurements,
        ObjectMap& local_targets,
        std::shared_ptr<VoxelSet> local_voxels)
    {
        // just to shut compiler up, this needs to be overwritten in the inherited class
        std::shared_ptr<AssociationVector> null;
        return null;
    }

    // Mahalanobis distance
    //
    // distance between point and distribution
    static double mahalanobis(Gaussian3D& distribution, Vector3& measurement);
    // squared distance between point and distribution
    static double sq_mahalanobis(Gaussian3D& distribution, Vector3& measurement);
    // mean distance between cluster of points and distribution
    static double mahalanobis(Gaussian3D& distribution, VoxelSet& points);

    // Bhattacharyya distance
    static double bhattacharyya(Gaussian3D& d1, Gaussian3D& d2);

    // Intersection_over_union between two clusters of voxels
    static double intersection_over_union(
        VoxelSet& measurement,
        VoxelSet& target);
    static double in_place_intersection(
        VoxelSet& measurement,
        VoxelSet& target,
        VoxelSet& intersection_set);
    static double in_place_union(
        VoxelSet& measurement,
        VoxelSet& target,
        VoxelSet& union_set);

protected:
    unsigned int quantise_double_likelihood(double val, unsigned int n_bins);
    double unquantise_int_likelihood(unsigned int q_val, unsigned int n_bins);

    unsigned int n_bins_ = 1000;

    unsigned int min_id_;
    unsigned int running_id_;
    unsigned int generate_new_id();

    bool use_local_reference_;

}; // class Object_Tracker_Base
} // namespace fusion
#endif // FUSION_OBJECT_TRACKER_BASE_H

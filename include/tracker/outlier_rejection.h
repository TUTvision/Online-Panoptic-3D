/* raiviol 10/2021
*/

#ifndef FUSION_OUTLIER_REJECTION_H
#define FUSION_OUTLIER_REJECTION_H

#include <data_structures.h>
#include <distributions.h>
#include <object_tracker_base.h>

#include <memory>
#include <random>
#include <iterator>
#include <algorithm>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

namespace fusion
{
// 3D Cartesian space for clustering
typedef bg::model::point<float, 3, bg::cs::cartesian> Point;
typedef std::pair<Point, voxblox::GlobalIndex> PointType;
typedef std::vector<PointType> DataVector;

// R* Tree for faster indexing in dbscan
typedef bgi::rtree< PointType, bgi::rstar<32> > RTree;

typedef std::unordered_map<int, VoxelSet> ClusterMap;
typedef VoxelUintMap LabelMap;

class Outlier_Rejection
{
public:
    Outlier_Rejection(){};
    Outlier_Rejection(
        unsigned int min_pts,
        double threshold)
        : min_pts_(min_pts),
          threshold_(threshold_){}

    virtual ~Outlier_Rejection() = default;

    virtual VoxelSet reject_outliers(VoxelSet& points)
    {
        VoxelSet null;
        return null;
    }
    virtual VoxelSet reject_outliers(VoxelSet& points, Gaussian3D& distribution)
    {
        VoxelSet null;
        return null;
    }

protected:
    unsigned int min_pts_;
    double threshold_;

}; // class Outlier_Rejection

class DBSCAN_OR : public Outlier_Rejection
{
public:
    DBSCAN_OR(unsigned int min_points, double epsilon)
        : Outlier_Rejection(min_points, epsilon){}
    ~DBSCAN_OR(){};

    /* Cluster a set of 3D points (voxel centers) with DBSCAN
     * return: unordered map<cluster_id, voxel_id's>
     */
    void run_clustering(
        VoxelSet& points,
        ClusterMap& clusters);

    /* Cluster a set of 3D points (voxel centers) with DBSCAN, and only accept
     * largest cluster as inliers
     * return: voxel id's in largest cluster
     */
    VoxelSet reject_outliers(VoxelSet& points) override;

protected:
    void build_rtree(VoxelSet& points, RTree& tree);

    void clustering_inner_loop(
        std::shared_ptr<DataVector> neighbors,
        std::shared_ptr<LabelMap> labeled_voxels,
        RTree& points,
        int cluster_id);

    std::shared_ptr<DataVector> rangeQuery_(
        RTree& points,
        PointType Q);

    int noise_label_ = 0;
    int undef_label_ = -1;

}; // class DBSCAN_OR

class Gaussian_OR : public Outlier_Rejection
{
public:
    Gaussian_OR(unsigned int min_points, double threshold)
        : Outlier_Rejection(min_points, threshold){}
    ~Gaussian_OR(){};

    // Infer gaussian distribution from a set of points, and
    // reject points based on Mahalanobis distance
    VoxelSet reject_outliers(VoxelSet& points) override;
    // Same as above, but allows user to define the distribution themselves
    VoxelSet reject_outliers(VoxelSet& points, Gaussian3D& distribution) override;

}; // Gaussian_OR
} // namespace fusion

#endif // FUSION_OUTLIER_REJECTION_H

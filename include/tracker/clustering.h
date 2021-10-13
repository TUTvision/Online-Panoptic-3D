/* raiviol 6/2021
*/

#ifndef FUSION_CLUSTERING_H
#define FUSION_CLUSTERING_H

#include <voxel_structures.h>

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
// 3D Cartesian space
typedef bg::model::point<float, 3, bg::cs::cartesian> Point;
typedef std::pair<Point, voxblox::GlobalIndex> PointType;
typedef std::vector<PointType> DataVector;

// test different indexes for optimal speed, r* is best for queries at least
// whats the effect of MaxElements ?
typedef bgi::rtree< PointType, bgi::rstar<32> > RTree;

typedef std::unordered_map<int, VoxelSet> ClusterMap;
typedef VoxelUintMap LabelMap;

class DBSCAN
{
public:
    DBSCAN(unsigned int min_points=10, double epsilon=1.5)
        : min_pts_(min_points), eps_(epsilon){}
    ~DBSCAN(){};

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
    VoxelSet reject_outliers(VoxelSet& points);

    void build_rtree(VoxelSet& points, RTree& tree);

protected:
    void clustering_inner_loop(
        std::shared_ptr<DataVector> neighbors,
        std::shared_ptr<LabelMap> labeled_voxels,
        RTree& points,
        int cluster_id);

    std::shared_ptr<DataVector> rangeQuery_(
        RTree& points,
        PointType Q);

    unsigned int min_pts_;
    double eps_;

    int noise_label_ = 0;
    int undef_label_ = -1;

}; // end class DBSCAN
} // end namespace fusion
#endif // FUSION_CLUSTERING_H

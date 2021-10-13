/* raiviol 6/2021
*/

#include <clustering.h>

#include <chrono>
#include <ros/console.h>

namespace fusion
{

/* PUBLIC FUNCTIONS
*/

void DBSCAN::run_clustering(
    VoxelSet& points,
    ClusterMap& clusters)
{
    unsigned int n_clusters = 0;

    RTree tree;
    build_rtree(points, tree);

    std::shared_ptr<LabelMap> labeled_voxels = std::make_shared<LabelMap>();

    for(auto const& P : tree)
    {
        // voxel found in label map -> is defined -> already processed in inner loop
        if( labeled_voxels->find(P.second) != labeled_voxels->end() )
        {
            continue;
        }

        // neighbourhood sans current point
        auto seed_set = rangeQuery_(tree, P);

        // cluster too small, label P as noise
        if( seed_set->size() < min_pts_ )
        {
            (*labeled_voxels)[P.second] = noise_label_;
            continue;
        }

        /*
        // remove P vs tolerate additional iteration?
        auto pt_it = seed_set->find(P);
        if( pt_it != seed_set->end() )
        {
            seed_set->erase(pt_it);
        }
        */

        // create new cluster
        ++n_clusters;

        (*labeled_voxels)[P.second] = n_clusters;

        clustering_inner_loop(
            seed_set,
            labeled_voxels,
            tree,
            n_clusters
        );
    }

    // form ClusterMap from results
    for( auto point_pair : *labeled_voxels )
    {
        unsigned int id = point_pair.second;

        // ignore noise
        if( id != noise_label_ )
        {
            auto voxel = point_pair.first;

            clusters[id].insert(voxel);
        }
    }
}

VoxelSet DBSCAN::reject_outliers(VoxelSet& points)
{
    //ROS_INFO("RUN DBSCAN");

    ClusterMap clusters;
    run_clustering(points, clusters);

    VoxelSet largest_cluster;
    unsigned int top_n = 0;

    for( auto cluster : clusters )
    {
        unsigned int n = cluster.second.size();
        if( n > top_n )
        {
            largest_cluster = cluster.second;
            top_n = n;
        }
    }
    return largest_cluster;
}

void DBSCAN::build_rtree(VoxelSet& points, RTree& rtree)
{
    for( auto P : points )
    {
        Point p(P[0],P[1],P[2]);

        PointType pt(p, P);

        rtree.insert(pt);
    }
}

/* PROTECTED FUNCTIONS
*/

void DBSCAN::clustering_inner_loop(
    std::shared_ptr<DataVector> neighbors,
    std::shared_ptr<LabelMap> labeled_voxels,
    RTree& points,
    int cluster_id)
{
    for( auto Q : *neighbors )
    {
        auto iter = labeled_voxels->find(Q.second);

        // if point is noise or undefined, define as border point
        // else, it has already been processed and can be ignored
        if( iter != labeled_voxels->end() && iter->second != noise_label_ )
        {
            continue;
        }
        (*labeled_voxels)[Q.second] = cluster_id;

        // if set of neighbors of a border point is over threshold,
        // it becomes a core point and its neighbors are added to seed set
        auto inner_neighbors = rangeQuery_(points, Q);
        if( inner_neighbors->size() >= min_pts_ )
        {
            clustering_inner_loop(
                inner_neighbors,
                labeled_voxels,
                points,
                cluster_id
            );
        }
    }
}

std::shared_ptr<DataVector> DBSCAN::rangeQuery_(
    RTree& rt,
    PointType Q)
{
    auto neighbors = std::make_shared<DataVector>();

    rt.query(bgi::satisfies(
            [&](PointType const& P)
            {
                return bg::distance(P.first, Q.first) < eps_;
            }
        ),
        std::back_inserter(*neighbors));

    return neighbors;
}

} // end namespace fusion

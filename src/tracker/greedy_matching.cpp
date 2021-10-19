
/* raiviol 8/2021
*/

#include<greedy_matching.h>

namespace fusion
{

Greedy_LAP::Greedy_LAP(
    unsigned int min_id,
    bool use_local_reference,
    double new_target_likelihood,
    std::string likelihood_metric,
    std::shared_ptr<VoxelMap> reference_map )
    : Object_Tracker_Base(min_id, use_local_reference),
      new_target_likelihood_(new_target_likelihood),
      likelihood_metric_(likelihood_metric),
      reference_map_(reference_map)
{
    if( likelihood_metric == "iou" && !reference_map)
    {
        throw std::invalid_argument("IoU computation requires reference voxel map.");
    }
    if( likelihood_metric != "iou" )
    {
        throw std::invalid_argument("Greedy matching is currently only implemented for IoU");
    }
}

std::shared_ptr<AssociationVector> Greedy_LAP::process_measurements(
    ObjectMap& measurements,
    ObjectMap& local_targets,
    std::shared_ptr<VoxelSet> local_voxels)
{
    IdVector measurement_ids;
    IdVector target_ids;

    for( auto m : measurements )
    {
        measurement_ids.push_back(m.first);
    }

    if( local_targets.size() > 0 )
    {
        target_ids.reserve(local_targets.size());

        for( auto id_object_pair : local_targets )
        {
            target_ids.push_back(id_object_pair.first);
        }
    }

    auto assignments = std::make_shared<AssociationVector>();

    if( measurement_ids.size() < 1 )
    {
        return assignments;
    }

    std::vector<std::thread> threads;

    unsigned int row_idx  = 0;

    for( auto measurement_id : measurement_ids )
    {
        Association_struct a;
        a.measurement_id = measurement_id;
        a.likelihood = 0;

        for( auto target_id : target_ids )
        {
            double iou = intersection_over_union(
                measurements[measurement_id]->points,
                local_targets[target_id]->points
            );

            if( iou > a.likelihood )
            {
                a.likelihood = iou;
                a.target_id = target_id;
            }
        }

        if( a.likelihood <= new_target_likelihood_)
        {
            a.target_id = generate_new_id();
            //a.likelihood = new_target_likelihood_;
        }

        assignments->push_back(a);
    }

    // TODO: fuse distributions if too much overlap?

    return assignments;
}

} // namespace fusion

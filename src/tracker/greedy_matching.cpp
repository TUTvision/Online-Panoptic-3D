
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
    std::shared_ptr<IdSet> local_reference,
    std::shared_ptr<VoxelSet> local_voxels)
{
    update_map();

    IdVector measurement_ids;
    IdVector targets;

    for( auto m : measurements )
    {
        measurement_ids.push_back(m.first);
    }

    std::sort(measurement_ids.begin(), measurement_ids.end(),
        [&measurements](unsigned int a, unsigned int b)
        {
            return measurements[a]->points.size() > measurements[b]->points.size();
        }
    );

    if( local_reference )
    {
        targets.reserve(local_reference->size());

        for( unsigned int id : *local_reference )
        {
            if( map_.count(id) > 0 )
            {
                targets.push_back(id);
            }
        }
    }
    else
    {
        for( auto pair : map_ )
        {
            // avoid mapping stuff id's
            unsigned int id = pair.first;
            if( id >= min_id_)
            {
                targets.push_back(pair.first);
            }
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

        for( auto target_id : targets )
        {
            VoxelSet target_points = map_[target_id]->points;

            if( local_voxels && local_voxels->size() > 0 )
            {
                // edits target_points in-place to only contain local voxels
                double u = in_place_union(map_[target_id]->points, *local_voxels, target_points);
            }

            double iou = intersection_over_union(
                measurements[measurement_id]->points,
                target_points
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

void Greedy_LAP::update_map()
{
    // when likelihood is computed as IoU, a reference map is used in order to
    // avoid overlapping targets

    std::unordered_set<unsigned int> cleared_ids;

    for( auto pair : *reference_map_ )
    {
        auto voxel = pair.first;
        auto id = pair.second.first;
        auto weight = pair.second.second;

        if( map_.count(id) == 0)
        {
            //std::shared_ptr<Object_struct> init =
            map_[id] = std::make_shared<Object_struct>();
        }

        // Clear voxels to avoid overlaps. Reference_map can also have only
        // recently updated id's, therefore this should be performed inside
        // the loop to not affect id's in map not found in the reference.
        else if( cleared_ids.count(id) == 0 )
        {
            map_[id]->points.clear();
            map_[id]->weights.clear();
            cleared_ids.insert(id);
        }

        map_[id]->points.insert(voxel);
        map_[id]->weights[voxel] = weight;
    }
}

void Greedy_LAP::update_map(
    ObjectMap& measurements,
    std::shared_ptr<AssociationVector> assignments)
{
    for( auto a : *assignments )
    {
        if( map_.count(a.target_id) == 0 )
        {
            auto init = std::make_shared<Object_struct>();
            init->distribution.init(
                measurements[a.measurement_id]->points.begin(),
                measurements[a.measurement_id]->points.end());

            map_[a.target_id] = init;
        }
        else
        {
            map_[a.target_id]->distribution.update(
                measurements[a.measurement_id]->points.begin(),
                measurements[a.measurement_id]->points.end());
        }
    }
}

} // namespace fusion

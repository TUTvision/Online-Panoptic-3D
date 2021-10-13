
/* raiviol 8/2021
*/

#include<hungarian_lap.h>

namespace fusion
{

H_LAP::H_LAP(
    unsigned int min_id,
    bool use_local_reference,
    bool normalise_likelihoods,
    double new_target_likelihood,
    std::string likelihood_metric,
    std::shared_ptr<VoxelMap> reference_map )
    : Object_Tracker_Base(min_id, use_local_reference),
      normalise_likelihoods_(normalise_likelihoods),
      new_target_likelihood_(new_target_likelihood),
      likelihood_metric_(likelihood_metric),
      reference_map_(reference_map)
{
    if( likelihood_metric_ == "iou" && !reference_map)
    {
        throw std::invalid_argument("IoU computation requires reference voxel map.");
    }
}

std::shared_ptr<AssociationVector> H_LAP::process_measurements(
    ObjectMap& measurements,
    std::shared_ptr<IdSet> local_reference,
    std::shared_ptr<VoxelSet> local_voxels)
{
    if( likelihood_metric_ == "iou")
    {
        update_map();
    }

    IdVector measurement_ids;
    IdVector targets;

    for( auto m : measurements )
    {
        measurement_ids.push_back(m.first);
    }

    if( local_reference && local_reference->size() > 0 )
    {
        targets.reserve(local_reference->size());

        for( unsigned int id : *local_reference )
        {
            if( map_.count(id) > 0 )
            {
                targets.push_back(id);
            }
        }
        //targets.insert(targets.end(), local_reference->begin(), local_reference->end());
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

    unsigned int n = ( measurement_ids.size() > targets.size() ) ?
        measurement_ids.size() : targets.size();

    auto assignments = std::make_shared<AssociationVector>();

    if( targets.size() < 1 )
    {
        return assignments;
    }

    // rows: measurement id, cols: target id, values are likelihoods of match
    // needs to be square, surplus likelihoods are set to zero to not affect assignement
    dlib::matrix<double> likelihood_mat = dlib::zeros_matrix<double>(n,n);

    std::vector<std::thread> threads;

    unsigned int row_idx  = 0;

    for( auto measurement_id : measurement_ids )
    {
        unsigned int col_idx = 0;

        for( auto target_id : targets )
        {
            if( measurements[measurement_id]->points.size() > 0 )
            {
                threads.push_back(
                    std::thread(
                        &H_LAP::likelihood_matrix_callback, this,
                        measurements[measurement_id],
                        map_[target_id],
                        local_voxels,
                        row_idx,
                        col_idx,
                        std::ref(likelihood_mat)
                    )
                );
            }
            ++col_idx;
        }
        ++row_idx;
    }

    Common::join_threads(threads);

    double norm = 1;
    double threshold = 1/(double)n;//targets.size();// new_target_likelihood_;

    if( normalise_likelihoods_ )
    {
        // likelihoods lie in [0, max]
        // divide by max -> range is [0, 1]

        norm = dlib::sum(likelihood_mat);// + new_target_likelihood_;
        likelihood_mat = (likelihood_mat - dlib::min(likelihood_mat)) / norm;
        //threshold = norm;
    }

    dlib::matrix<int> int_likelihood_mat = dlib::zeros_matrix<int>(n,n);

    for( unsigned int i = 0 ; i < n ; ++i )
    {
        for( unsigned int j = 0 ; j < n ; ++j )
        {
            if( likelihood_mat(i,j) > 0 )
            {
                //ROS_INFO("Likelihood: %f", likelihood_mat(i,j));
            }

            int_likelihood_mat(i,j) =
                quantise_double_likelihood(likelihood_mat(i,j), n_bins_);
        }
    }

    std::vector<long> assignment_vec = dlib::max_cost_assignment(int_likelihood_mat);

    unsigned int measurement_idx = 0;
    for( unsigned int assigned_target_idx : assignment_vec )
    {
        if( measurement_idx >= measurements.size() )
        {
            break;
        }

        Association_struct a;

        double likelihood =
            likelihood_mat(measurement_idx, assigned_target_idx);

        a.measurement_id = measurement_ids[measurement_idx];

        if( likelihood > threshold )
        {
            a.target_id = targets[assigned_target_idx];
        }
        else
        {
            a.target_id = generate_new_id();
            likelihood = threshold;
        }

        a.likelihood = likelihood;

        assignments->push_back(a);

        ++measurement_idx;
    }

    if( likelihood_metric_ != "iou" )
    {
        update_map(measurements, assignments);
    }

    // TODO: fuse distributions if too much overlap?

    return assignments;
}

void H_LAP::likelihood_matrix_callback(
    std::shared_ptr<Object_struct> measurement,
    std::shared_ptr<Object_struct> target,
    std::shared_ptr<VoxelSet> local_voxels,
    unsigned int measurement_index,
    unsigned int target_index,
    dlib::matrix<double>& likelihood_matrix)
{
    double likelihood = 0;
    double distance = std::numeric_limits<double>::infinity();

    if( likelihood_metric_ == "bh" )
    {
        distance = bhattacharyya(
            measurement->distribution,
            target->distribution
        );
    }
    else if( likelihood_metric_ == "mh" )
    {
        distance = mahalanobis(
            target->distribution,
            measurement->points
        );
    }
    else
    {
        VoxelSet target_points = target->points;

        if( local_voxels && local_voxels->size() > 0 )
        {
            // edits target_points in-place to only contain local voxels
            double u = in_place_union(target->points, *local_voxels, target_points);
        }

        likelihood = intersection_over_union(
            measurement->points,
            target_points
        );
    }

    // convert distance to likelihood
    if( likelihood == 0 )
    {
        if( normalise_likelihoods_ )
        {
            // linear
            //likelihood = -distance;

            // the further an object is, the more uniform likelihood it has to others
            // add small number to not divide by zero, otherwise normalisation might fail
            likelihood = 1 / (distance + 1e-10);
        }
        else
        {
            // range [0, 1]
            likelihood = 1 / (1 + distance);
        }
    }

    likelihood_matrix(measurement_index, target_index) = likelihood;
}

void H_LAP::update_map()
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

void H_LAP::update_map(
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

/* raiviol 8/2021
*/

#include<hungarian_lap.h>

namespace fusion
{

H_LAP::H_LAP(
    unsigned int min_id,
    bool use_local_reference,
    bool normalise_likelihoods,
    double association_threshold,
    std::string likelihood_metric)
    : Object_Tracker_Base(min_id, use_local_reference),
      normalise_likelihoods_(normalise_likelihoods),
      association_threshold_(association_threshold),
      likelihood_metric_(likelihood_metric)
{
}

std::shared_ptr<AssociationVector> H_LAP::process_measurements(
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

    const unsigned int n = ( measurement_ids.size() > target_ids.size() ) ?
        measurement_ids.size() : target_ids.size();

    auto assignments = std::make_shared<AssociationVector>();

    if( n < 1 )
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

        for( auto target_id : target_ids )
        {
            if( measurements[measurement_id]->points.size() > 0 )
            {
                threads.push_back(
                    std::thread(
                        &H_LAP::likelihood_matrix_callback, this,
                        measurements[measurement_id],
                        local_targets[target_id],
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

    double threshold = association_threshold_;

    dlib::matrix<int> int_likelihood_mat = dlib::zeros_matrix<int>(n,n);

    if( normalise_likelihoods_ )
    {
        threshold /= (double)measurements.size();

        std::vector<double> sums(n, 0);

        for( unsigned int i = 0 ; i < n ; ++i )
        {
            for( unsigned int j = 0 ; j < n ; ++j )
            {
                sums[i] += likelihood_mat(i,j);
            }
        }
        for( unsigned int i = 0 ; i < n ; ++i )
        {
            for( unsigned int j = 0 ; j < n ; ++j )
            {
                likelihood_mat(i,j) = (sums[i] > 0) ? likelihood_mat(i,j) / sums[i] : 0;
            }
        }
    }

    for( unsigned int i = 0 ; i < n ; ++i )
    {
        for( unsigned int j = 0 ; j < n ; ++j )
        {
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
            a.target_id = target_ids[assigned_target_idx];
        }
        else
        {
            a.target_id = generate_new_id();
        }

        a.likelihood = likelihood;

        assignments->push_back(a);

        ++measurement_idx;
    }

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
        likelihood = intersection_over_union(
            measurement->points,
            target->points
        );
    }

    // convert distance to likelihood
    if( likelihood == 0 )
    {
        if( normalise_likelihoods_ )
        {
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

} // namespace fusion

/* raiviol 8/2021
*/

#include <object_tracker_base.h>

namespace fusion
{

Object_Tracker_Base::Object_Tracker_Base(unsigned int min_id, bool use_local_reference)
    : min_id_(min_id),
      running_id_(min_id),
      use_local_reference_(use_local_reference)
{

}

double Object_Tracker_Base::intersection_over_union(
    VoxelSet& measurement,
    VoxelSet& target)
{
    VoxelSet intersection_set;
    VoxelSet union_set;

    double I = in_place_intersection(
        measurement,
        target,
        intersection_set);

    double U = in_place_union(
        measurement,
        target,
        intersection_set);

    return (U > 0) ? I / U : 0;
}

double Object_Tracker_Base::in_place_union(
    VoxelSet& measurement,
    VoxelSet& target,
    VoxelSet& union_set)
{
    union_set.insert(measurement.begin(), measurement.end());
    union_set.insert(target.begin(), target.end());

    return union_set.size();
}

double Object_Tracker_Base::in_place_intersection(
    VoxelSet& measurement,
    VoxelSet& target,
    VoxelSet& intersection_set)
{
    VoxelSet* iter_set;
    VoxelSet* other_set;

    if( measurement.size() < target.size() )
    {
        iter_set  = &measurement;
        other_set = &target;
    }
    else
    {
        iter_set  = &measurement;
        other_set = &target;
    }

    for( auto vx : *iter_set )
    {
        if( other_set->find(vx) != other_set->end() )
        {
            intersection_set.insert(vx);
        }
    }

    return intersection_set.size();
}

double Object_Tracker_Base::bhattacharyya(Gaussian3D& d1, Gaussian3D& d2)
{
    // Bhattacharyya distance between two 3d multivariate normal distributions

    Vector3 m_diff = d1.mean - d2.mean;
    Matrix3 m_cov = (d1.covariance + d2.covariance) / 2;

    // squared Mahanolobis distance ?
    double mhsq = m_diff.adjoint() * m_cov.inverse() * m_diff;

    auto det1  = d1.covariance.determinant();
    auto det2  = d2.covariance.determinant();
    auto det_m = m_cov.determinant();

    return mhsq / 8 + 0.5 * std::log( det_m / std::sqrt(det1*det2) );
}

double Object_Tracker_Base::mahalanobis(
    Gaussian3D& distribution,
    VoxelSet& points )
{
    double sum = 0;
    unsigned int n = 0;

    for( auto p : points )
    {
        Vector3 p_float = p.cast<float>();
        sum += mahalanobis(distribution, p_float);
        ++n;
    }

    double distance = (n > 0) ? sum/n : std::numeric_limits<double>::infinity();;

    return distance;
}

double Object_Tracker_Base::mahalanobis(
    Gaussian3D& distribution,
    Vector3& measurement)
{
    return std::sqrt(sq_mahalanobis(distribution, measurement));
}

double Object_Tracker_Base::sq_mahalanobis(
    Gaussian3D& distribution,
    Vector3& point)
{
    Vector3 m_diff = point - distribution.mean;
    return m_diff.adjoint() * distribution.covariance.inverse() * m_diff;
}

unsigned int Object_Tracker_Base::quantise_double_likelihood(double val, unsigned int n_bins)
{
    unsigned int q_val = n_bins*val;
    return q_val;
}

double Object_Tracker_Base::unquantise_int_likelihood(unsigned int q_val, unsigned int n_bins)
{
    double val = (double)q_val / n_bins;
    return val;
}

unsigned int Object_Tracker_Base::generate_new_id()
{
    ++running_id_;
    return running_id_;
}

} // namespace fusion

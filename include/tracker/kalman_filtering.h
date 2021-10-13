/* raiviol 8/2021
*/

#ifndef FUSION_KALMAN_H
#define FUSION_KALMAN_H

#include <distributions.h>

#include <Eigen/Core>
#include <voxblox/core/tsdf_map.h>

#include <random>

namespace fusion
{
struct GaussianModel3D
{
    Matrix3 A;
    Matrix3 B;
};

class KF
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KF( GaussianModel3D dyn_model,
        GaussianModel3D meas_model,
        Gaussian3D prior)
        : F(dyn_model.A),
          Q(dyn_model.B),
          H(meas_model.A),
          R(meas_model.B),
          distribution(prior)
    {
    }
    ~KF(){}

    Gaussian3D distribution;

    void predict()
    {
        distribution.mean = F * distribution.mean;
        distribution.covariance = F * distribution.covariance * F.adjoint() + Q;
    }

    void update(Vector3 m)
    {
        Matrix3 S = H * distribution.covariance * H.adjoint() + R;
        Matrix3 K = distribution.covariance * H.adjoint() * S.inverse();

        Matrix3 I = Matrix3::Identity();

        distribution.mean += K * ( m - H*distribution.mean );
        distribution.covariance = ( I - K*H ) * distribution.covariance;
    }

    Vector3 sample()
    {
        return distribution.sample();
    }

protected:
    // dynamic model
    Matrix3 F;
    Matrix3 Q;

    // measurement model
    Matrix3 H;
    Matrix3 R;
};

}; // namespace fusion

#endif // FUSION_KALMAN_H

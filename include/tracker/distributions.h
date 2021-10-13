/* raiviol 5/2021
*/

#ifndef FUSION_DISTRIBUTIONS_H
#define FUSION_DISTRIBUTIONS_H

#include <random>
#include <Eigen/Core>
#include <voxblox/core/tsdf_map.h>

namespace fusion
{
    typedef Eigen::Matrix<voxblox::FloatingPoint, 3, 1> Vector3;
    typedef Eigen::Matrix<voxblox::FloatingPoint, 3, 3> Matrix3;

    static std::mt19937 gen_{ std::random_device{}() };
    static std::normal_distribution<> dist_;

    struct Gaussian3D
    {
        bool undef = true;

        unsigned int n_points = 0;

        Vector3 mean;
        Matrix3 covariance;

        // batch update, no prior
        template<typename Iter>
        void init(Iter begin, Iter end)
        {
            n_points = std::distance(begin, end);

            if( n_points > 0 )
            {
                Eigen::MatrixXf samples(3, n_points);

                unsigned int i = 0;
                while( begin != end )
                {
                    auto point = begin->template cast<float>();

                    samples.col(i) = point;

                    ++i;
                    ++begin;
                }

                init(samples);
            }
        }

        void init(Eigen::MatrixXf& samples)
        {
            if( n_points > 0 )
            {
                mean = samples.rowwise().mean();
                Eigen::MatrixXf centered = samples.colwise() - mean;
                covariance = (centered * centered.adjoint() ) / n_points;

                undef = false;
            }
        }

        // batch update with prior using iterators
        template<typename Iter>
        void update(Iter begin, Iter end)
        {
            Gaussian3D tmp_dist;
            tmp_dist.init(begin, end);

            fuse(tmp_dist);
        }

        // batch update with prior, k measurements
        void update(voxblox::LongIndexVector& points)
        {
            // This only works for stationary distributions.

            Gaussian3D tmp_dist;
            tmp_dist.init(points.begin(), points.end());

            fuse(tmp_dist);
        }

        // merge two gaussians, faster than recursive update,
        // since no measurements have to be processed
        void fuse(Gaussian3D& dist)
        {
            if( undef )
            {
                  n_points = dist.n_points;
                      mean = dist.mean;
                covariance = dist.covariance;

                undef = false;
            }
            else
            {
                double n1 = n_points;
                double n2 = dist.n_points;

                mean = ( n1*mean + n2*dist.mean ) / ( n1 + n2 );

                double w1 = std::pow(n1, 2) / std::pow(n1 + n2, 2);
                double w2 = std::pow(n2, 2) / std::pow(n1 + n2, 2);

                covariance = w1*covariance + w2*dist.covariance;

                n_points = n1 + n2;
            }
        }

        // take one sample of 3D multivariate Gaussian (axes can be correlated)
        Vector3 sample()
        {
            Matrix3 normTransform;

            // if covariance is positive semi-definite, spectral decomposition
            // has to be used, otherwise cholesky decomposition is enough

            Eigen::LLT<Matrix3> cholSolver(covariance);

            if (cholSolver.info()==Eigen::Success)
            {
                normTransform = cholSolver.matrixL();
            }
            else
            {
                Eigen::SelfAdjointEigenSolver<Matrix3> eigenSolver(covariance);

                normTransform = eigenSolver.eigenvectors()
                       * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
            }

            Vector3 rand;
            rand.unaryExpr([&](float x) { return dist_(gen_); });

            Vector3 sample = mean + normTransform * rand;

            return sample;
        }
    };
}; // end namespace fusion

#endif // FUSION_DISTRIBUTIONS_H

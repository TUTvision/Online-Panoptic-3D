/* raiviol 5/2021
*/

#ifndef FUSION_SIMPLE_INTEGRATOR_H
#define FUSION_SIMPLE_INTEGRATOR_H

#include <integrator_base.h>

#include <voxblox/core/tsdf_map.h>

namespace fusion
{
class SimpleTsdfIntegrator : public TsdfIntegratorBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SimpleTsdfIntegrator(
        const TsdfIntegratorBase::Config& config,
        voxblox::Layer<voxblox::TsdfVoxel>* layer,
        fusion::Config fusion_config,
        ros::Publisher confirm_pub)
            : TsdfIntegratorBase(config, layer, fusion_config, confirm_pub){}

    void integratePointCloud(const voxblox::Transformation& T_G_C,
                             const voxblox::Pointcloud& points_C,
                             const voxblox::Colors& colors,
                             const bool freespace_points = false);
protected:
    void integrateFunction(const voxblox::Transformation& T_G_C,
                           const voxblox::Pointcloud& points_C,
                           const voxblox::Colors& colors,
                           const bool freespace_points,
                           voxblox::ThreadSafeIndex* index_getter);

    std::mutex update_point_mtx_;
    std::mutex add_voxel_mtx_;

};
} // end namespace fusion
#endif // FUSION_SIMPLE_INTEGRATOR_H

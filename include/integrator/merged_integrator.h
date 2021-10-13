
#ifndef FUSION_MERGED_INTEGRATOR_H
#define FUSION_MERGED_INTEGRATOR_H

#include <integrator_base.h>

#include <voxblox/core/tsdf_map.h>

namespace fusion
{
class MergedTsdfIntegrator : public TsdfIntegratorBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MergedTsdfIntegrator(
        const TsdfIntegratorBase::Config& config,
        voxblox::Layer<voxblox::TsdfVoxel>* layer,
        fusion::Config fusion_config,
        ros::Publisher confirm_pub)
            : TsdfIntegratorBase(config, layer, fusion_config, confirm_pub){}

    void integratePointCloud(
        const voxblox::Transformation& T_G_C,
        const voxblox::Pointcloud& points_C,
        const voxblox::Colors& colors,
        const bool freespace_points = false);

protected:
    void bundleRays(
        const voxblox::Transformation& T_G_C,
        const voxblox::Pointcloud& points_C,
        const bool freespace_points,
        voxblox::ThreadSafeIndex* index_getter,
        voxblox::LongIndexHashMapType<voxblox::AlignedVector<size_t>>::type* voxel_map,
        voxblox::LongIndexHashMapType<voxblox::AlignedVector<size_t>>::type* clear_map);

    void integrateRays(
        const voxblox::Transformation& T_G_C,
        const voxblox::Pointcloud& points_C,
        const voxblox::Colors& colors,
        bool enable_anti_grazing,
        bool clearing_ray,
        const voxblox::LongIndexHashMapType<voxblox::AlignedVector<size_t>>::type& voxel_map,
        const voxblox::LongIndexHashMapType<voxblox::AlignedVector<size_t>>::type& clear_map);

    void integrateVoxels(
        const voxblox::Transformation& T_G_C,
        const voxblox::Pointcloud& points_C,
        const voxblox::Colors& colors,
        bool enable_anti_grazing,
        bool clearing_ray,
        const voxblox::LongIndexHashMapType<voxblox::AlignedVector<size_t>>::type& voxel_map,
        const voxblox::LongIndexHashMapType<voxblox::AlignedVector<size_t>>::type& clear_map,
        size_t thread_idx);

    void integrateVoxel(
        const voxblox::Transformation& T_G_C,
        const voxblox::Pointcloud& points_C,
        const voxblox::Colors& colors,
        bool enable_anti_grazing,
        bool clearing_ray,
        const std::pair<voxblox::GlobalIndex, voxblox::AlignedVector<size_t>>& kv,
        const voxblox::LongIndexHashMapType<voxblox::AlignedVector<size_t>>::type& voxel_map);

    std::mutex update_point_mtx_;
};
} // namespace fusion
#endif // FUSION_MERGED_INTEGRATOR_H

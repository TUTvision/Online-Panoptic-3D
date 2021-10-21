/* raiviol 5/2021
*/

#ifndef FUSION_FAST_INTEGRATOR_H
#define FUSION_FAST_INTEGRATOR_H

#include <integrator_base.h>

#include <voxblox/core/tsdf_map.h>

namespace fusion
{
class FastTsdfIntegrator : public TsdfIntegratorBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FastTsdfIntegrator(
        const TsdfIntegratorBase::Config& config,
        voxblox::Layer<voxblox::TsdfVoxel>* layer,
        fusion::Config fusion_config,
        ros::Publisher confirm_pub)
            : TsdfIntegratorBase(config, layer, fusion_config, confirm_pub){}

    void integratePointCloud(
        const voxblox::Transformation& T_G_C,
        const voxblox::Pointcloud& points_C,
        const voxblox::Colors& colors,
        const bool freespace_points = false
    );

protected:
    void integrateFunction(
        const voxblox::Transformation& T_G_C,
        const voxblox::Pointcloud& points_C,
        const voxblox::Colors& colors,
        const bool freespace_points,
        voxblox::ThreadSafeIndex* index_getter
    );

    std::mutex update_point_mtx_;
    std::mutex add_voxel_mtx_;

private:
    /**
       * Two approximate sets are used below. The limitations of these sets are
       * outlined in approx_hash_array.h, but in brief they are thread safe and very
       * fast, but have a small chance of returning false positives and false
       * negatives. As rejecting a ray or integrating an uninformative ray are not
       * very harmful operations this trade-off works well in this integrator.
       */

      /**
       * uses 2^20 bytes (8 megabytes) of ram per tester
       * A testers false negative rate is inversely proportional to its size
       */
      static constexpr size_t masked_bits_ = 20;
      /**
       * only needs to zero the above 8mb of memory once every 10,000 scans
       * (uses an additional 80,000 bytes)
       */
      static constexpr size_t full_reset_threshold_ = 10000;

      /**
       * Voxel start locations are added to this set before ray casting. The ray
       * casting only occurs if no ray has been cast from this location for this
       * scan.
       */
      voxblox::ApproxHashSet<
        masked_bits_,
        full_reset_threshold_,
        voxblox::GlobalIndex,
        voxblox::LongIndexHash>
          start_voxel_approx_set_;

      /**
       * This set records which voxels a scans rays have passed through. If a ray
       * moves through max_consecutive_ray_collisions voxels in a row that have
       * already been seen this scan, it is deemed to be adding no new information
       * and the casting stops.
       */
      voxblox::ApproxHashSet<
        masked_bits_,
        full_reset_threshold_,
        voxblox::GlobalIndex,
        voxblox::LongIndexHash>
          voxel_observed_approx_set_;

      /// Used in terminating the integration early if it exceeds a time limit.
      std::chrono::time_point<std::chrono::steady_clock> integration_start_time_;
};
} // namespace fusion
#endif // FUSION_FAST_INTEGRATOR_H

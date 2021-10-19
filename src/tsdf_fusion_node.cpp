/* raiviol 5/2021
*/

#include <config.h>
#include <integrator_base.h>
#include <simple_integrator.h>
#include <merged_integrator.h>
#include <fast_integrator.h>

#include <memory>

#include "ros/ros.h"
#include <ros/console.h>
#include <gflags/gflags.h>

#include "voxblox_ros/ros_params.h"
#include <voxblox_ros/tsdf_server.h>
#include <voxblox/integrator/tsdf_integrator.h>

namespace fusion
{
class TsdfServer: public voxblox::TsdfServer
{
public:
    TsdfServer(
        const ros::NodeHandle& nh,
        const ros::NodeHandle& nh_private,
        ros::Publisher confirm_pub)
        : voxblox::TsdfServer(nh, nh_private)
    {
        const TsdfIntegratorBase::Config& integrator_config =
            voxblox::getTsdfIntegratorConfigFromRosParam(nh_private);

        fusion::Config fusion_config = parse_config(nh_private);

        std::string method("merged");
        nh_private_.param("method", method, method);

        if (method.compare("simple") == 0)
        {
            tsdf_integrator_.reset(new SimpleTsdfIntegrator(
                integrator_config, tsdf_map_->getTsdfLayerPtr(), fusion_config, confirm_pub));
        }
        else if (method.compare("merged") == 0)
        {
            tsdf_integrator_.reset(new MergedTsdfIntegrator(
                integrator_config, tsdf_map_->getTsdfLayerPtr(), fusion_config, confirm_pub));
        }
        else if (method.compare("fast") == 0)
        {
            tsdf_integrator_.reset(new FastTsdfIntegrator(
                integrator_config, tsdf_map_->getTsdfLayerPtr(), fusion_config, confirm_pub));
        }
        else // fallback to default (simple integrator)
        {
            tsdf_integrator_.reset(new SimpleTsdfIntegrator(
                integrator_config, tsdf_map_->getTsdfLayerPtr(), fusion_config, confirm_pub));
        }

        // Advertise services.
        save_segmentation_srv_ = nh_private_.advertiseService(
            "save_segmentation",
            &fusion::TsdfIntegratorBase::createSegmentationCallback,
            dynamic_cast<fusion::TsdfIntegratorBase*>(&(*tsdf_integrator_))
        );

        // inform that integrator is ready to revieve data
        //dynamic_cast<fusion::TsdfIntegratorBase*>(&(*tsdf_integrator_))->send_confirmation_message(true);
    }
    ~TsdfServer(){};

protected:
    ros::ServiceServer save_segmentation_srv_;

    fusion::Config parse_config(const ros::NodeHandle& nh_private)
    {
        fusion::Config config;

        nh_private_.param(
            "tracker_type",
            config.tracker_type,
            config.tracker_type);
        nh_private_.param(
            "normalise_likelihoods",
            config.normalise_likelihoods,
            config.normalise_likelihoods);
        nh_private_.param(
            "only_process_local_voxels",
            config.only_process_local_voxels,
            config.only_process_local_voxels);
        nh_private_.param(
            "tracker_type",
            config.use_local_reference,
            config.use_local_reference);
        nh_private_.param(
            "likelihood_metric",
            config.likelihood_metric,
            config.likelihood_metric);
        nh_private_.param(
            "confidence_threshold",
            config.confidence_threshold,
            config.confidence_threshold);
        nh_private_.param(
            "association_threshold",
            config.association_threshold,
            config.association_threshold);
        nh_private_.param(
            "weighting_strategy",
            config.weighting_strategy,
            config.weighting_strategy);

        nh_private_.param(
            "perform_outlier_rejection",
            config.perform_outlier_rejection,
            config.perform_outlier_rejection);
        nh_private_.param(
            "outlier_rejection_method",
            config.outlier_rejection_method,
            config.outlier_rejection_method);
        nh_private_.param(
            "outlier_threshold",
            config.outlier_threshold,
            config.outlier_threshold);
        nh_private_.param(
            "cluster_min_points",
            config.cluster_min_points,
            config.cluster_min_points);
        nh_private_.param(
            "dbscan_epsilon",
            config.dbscan_epsilon,
            config.dbscan_epsilon);

        nh_private_.param(
            "infer_distribution",
            config.infer_distribution,
            config.infer_distribution);
        nh_private_.param(
            "sample_measurements",
            config.sample_measurements,
            config.sample_measurements);
        nh_private_.param(
            "max_samples",
            config.max_samples,
            config.max_samples);

        nh_private_.param(
            "output_name",
            config.output_name,
            config.output_name);
        nh_private_.param(
            "save_timings",
            config.save_timings,
            config.save_timings);
        nh_private_.param(
            "visualise_global_ids",
            config.visualise_global_ids,
            config.visualise_global_ids);
        nh_private_.param(
            "tsdf_voxel_size",
            config.voxel_size,
            config.voxel_size);

        return config;
    }
}; // class TsdfServer
} // namespace fusion

int main(int argc, char **argv)
{
    ros::init(argc, argv, "voxblox_panoptic_fusion");
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, false);
    google::InstallFailureSignalHandler();
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Publisher fusion_confirm_pub =
        nh.advertise<std_msgs::Bool>("voxel_fusion_confirm", 1000);

    fusion::TsdfServer node(nh, nh_private, fusion_confirm_pub);

    ros::spin();

    return 0;
}

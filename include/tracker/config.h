/* raiviol 5/2021
*/

#ifndef FUSION_CONFIG_H
#define FUSION_CONFIG_H

#include <string>

namespace fusion
{
    struct Config {
         /* tracker */
        std::string tracker_type       = "h_lap"; //"greedy";
        bool normalise_likelihoods     = true;
        bool only_process_local_voxels = true;
        bool use_local_reference       = true;
        std::string likelihood_metric  = "iou";
        double confidence_threshold    = 0.5;
        double association_threshold   = 0;
        std::string weighting_strategy = "tsdf";

        /* outlier rejection */
        bool perform_outlier_rejection       = false;
        std::string outlier_rejection_method = "dbscan"; //"gaussian";
        double outlier_threshold             = 6.2514;   // 90% quantile
        double cluster_min_points            = 10;
        double dbscan_epsilon                = 1.5;

        /* data procesing */
        bool infer_distribution  = false;
        bool sample_measurements = false;
        double max_samples       = 1000;

        /* output */
        std::string output_name   = "3d_panoptic";
        bool save_timings         = true;
        bool visualise_global_ids = true;
        double voxel_size;
    };
}; // end namespace fusion

#endif // FUSION_CONFIG_H

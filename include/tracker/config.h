/* raiviol 5/2021
*/

#ifndef FUSION_CONFIG_H
#define FUSION_CONFIG_H

#include <string>

namespace fusion
{
    struct Config {

        std::string labels_filename = "3d_panoptic";
        bool visualise_global_ids = true;

        std::string voxel_weighting = "constant";

        int max_samples = 1000;
        bool outlier_rejection = true;

        std::string likelihood_metric = "iou";
        double confidence_threshold = 0.5;
        double iou_threshold = 0.2;

        // minimum number of points (voxel centers) considered as a cluster
        double dbscan_min_points = 10;

        // voxels are in a normalised grid, perpendicular distances are 1
        // -> if we want to allow diagonal neighbors, epsilon sohuld be over sqrt(2)
        double dbscan_epsilon = 1.5;
    };
}; // end namespace fusion

#endif // FUSION_CONFIG_H

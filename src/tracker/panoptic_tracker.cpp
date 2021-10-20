
/* raiviol 8/2021
*/

#include<panoptic_tracker.h>

namespace fusion
{
PanopticTracker::PanopticTracker()
    : PanopticTracker(fusion::Config())
{
}
PanopticTracker::PanopticTracker(const fusion::Config& fusion_config)
    : config_(fusion_config)
{
    ids = std::make_shared<IdMap>();
    map = std::make_shared<VoxelMap>();
    local_voxels_ = std::make_shared<VoxelSet>();

    if( config_.perform_outlier_rejection )
    {
        if( config_.outlier_rejection_method == "dbscan" )
        {
            outlier_rejection_.reset(
                new DBSCAN_OR(
                    config_.cluster_min_points,
                    config_.dbscan_epsilon)
            );
            outlier_rejection_ =
                std::dynamic_pointer_cast<DBSCAN_OR>(outlier_rejection_);
        }
        else
        {
            config_.infer_distribution = true;

            outlier_rejection_.reset(
                new Gaussian_OR(
                    config_.cluster_min_points,
                    config_.outlier_threshold)
            );
            outlier_rejection_ =
                std::dynamic_pointer_cast<Gaussian_OR>(outlier_rejection_);
        }
    }

    // distribution-based metrics require a gaussian distribution
    if( config_.likelihood_metric != "iou" )
    {
        config_.infer_distribution = true;
    }

    if( config_.tracker_type == "greedy" )
    {
        tracker_.reset(
            new Greedy_LAP(
                N_STUFF_CLASSES,
                config_.normalise_likelihoods,
                config_.association_threshold,
                config_.likelihood_metric,
                map)
        );
        tracker_ = std::dynamic_pointer_cast<Greedy_LAP>(tracker_);
    }
    else
    {
        tracker_.reset(
            new H_LAP(
                N_STUFF_CLASSES,
                config_.use_local_reference,
                config_.normalise_likelihoods,
                config_.association_threshold,
                config_.likelihood_metric,
                map)
        );
        tracker_ = std::dynamic_pointer_cast<H_LAP>(tracker_);
    }
}

void PanopticTracker::add_measurement(
    unsigned int id,
    unsigned int category,
    Voxel point,
    double weight)
{
    // NOT THREAD SAFE

    if( Common::val_found_in_id_arr(category, STUFF_CLASS_IDX, N_STUFF_CLASSES) )
    {
        process_stuff_measurement(id, category,  point, weight);
    }
    else if( Common::val_found_in_id_arr(category, STATIC_OBJECT_IDX, N_STATIC_CLASSES) )
    {
        process_static_measurement(id, category,  point, weight);
    }
    else if( Common::val_found_in_id_arr(category, DYNAMIC_OBJECT_IDX, N_DYNAMIC_CLASSES) )
    {
        process_dynamic_measurement(id, category,  point, weight);
    }

    if( config_.only_process_local_voxels )
    {
        local_voxels_->insert(point);
    }
}

void PanopticTracker::update_map()
{
    using std::chrono::high_resolution_clock;
    using std::chrono::duration;

    if( measurements_.size() == 0 ){return;}

    auto t1 = high_resolution_clock::now();
    auto t1_total = t1;

    if( config_.sample_measurements || config_.infer_distribution )
    {
        std::vector<std::thread> sampling_threads;

        for( auto m : measurements_ )
        {
            sampling_threads.push_back(
                std::thread(
                    &PanopticTracker::sampling_callback, this,
                    m.second
                )
            );
        }

        Common::join_threads(sampling_threads);
    }

    if( config_.perform_outlier_rejection )
    {
        std::vector<std::thread> outlier_rejection_threads;

        for( auto m : measurements_ )
        {
            outlier_rejection_threads.push_back(
                std::thread(
                    &PanopticTracker::outlier_rejection_callback, this,
                    m.second
                )
            );
        }

        Common::join_threads(outlier_rejection_threads);
    }

    auto t2 = high_resolution_clock::now();

    duration<double> tdiff = t2 - t1;
    ROS_INFO("        update_ids: preprocess (s): %f", tdiff.count());
    save_time(tdiff, preprocess_times_);

    t1 = high_resolution_clock::now();

    ObjectMap local_reference;

    if( config_.use_local_reference )
    {
        for( auto p : *local_voxels_ )
        {
            if( map->count(p) > 0 )
            {
                unsigned int id = (*map)[p].first;

                if( !Common::val_found_in_id_arr(id, STUFF_CLASS_IDX, N_STUFF_CLASSES) )
                {
                    auto target_struct_it = local_reference.find(id);
                    if( target_struct_it == local_reference.end() )
                    {
                        auto init_struct = std::make_shared<Object_struct>();
                        local_reference[id] = init_struct;
                    }
                    local_reference[id]->points.insert(p);
                }
            }
        }
    }

    auto associations =
        tracker_->process_measurements(measurements_, local_reference, local_voxels_);

    if( associations )
    {
        for( auto a : *associations )
        {
            fuse_callback(
                measurements_[a.measurement_id],
                a.target_id
            );
        }
    }
    else
    {
        ROS_WARN("Association vector is nullptr, likely because Tracker_base::process_measurements was called.");
    }
    measurements_.clear();
    local_voxels_->clear();

    t2 = high_resolution_clock::now();
    auto t2_total = t2;

    tdiff = t2 - t1;
    ROS_INFO("        update_ids: data association (s): %f", tdiff.count());
    save_time(tdiff, association_times_);

    tdiff = t2_total - t1_total;
    ROS_INFO("        update_ids: total (s): %f", tdiff.count());
    save_time(tdiff, tracking_times_);
}

bool PanopticTracker::save_segmentation(double fusion_av)
{
    std::ofstream output_fs;
    std::ofstream colormap_fs;

    std::string txt_output_file = config_.output_name + ".txt";
    std::string color_map_file  = config_.output_name + "_color_map.txt";

    output_fs.open(txt_output_file);
    colormap_fs.open(color_map_file);

    std::unordered_set<unsigned int> processed;

    if (output_fs.is_open())
    {
        for( auto vx : *map )
        {
            Voxel voxel = vx.first;
            unsigned int id = vx.second.first;
            unsigned int category = (*ids)[id]->get_category(config_.confidence_threshold);
            double confidence = (*ids)[id]->confidence;

            std::string id_str  = std::to_string(id);
            std::string cat_str = std::to_string(category);
            std::string conf_str = std::to_string(confidence);

            if( processed.count(id) == 0 )
            {
                voxblox::Color color = (*ids)[id]->color;

                std::string color_str =
                    std::to_string(color.r) + " " +
                    std::to_string(color.g) + " " +
                    std::to_string(color.b);

                std::string line =
                  color_str + " " +
                  id_str + " " +
                  cat_str + " " +
                  conf_str + "\n";

                if( colormap_fs.is_open() )
                {
                    colormap_fs << line;
                }
                else
                {
                    ROS_INFO("Could not open file %s", color_map_file.c_str());
                }

                processed.insert(id);
            }

            voxblox::Point voxel_center =
                voxblox::getCenterPointFromGridIndex(voxel, config_.voxel_size);

            std::string point_str =
                std::to_string(voxel_center[0]) + " " +
                std::to_string(voxel_center[1]) + " " +
                std::to_string(voxel_center[2]);

            std::string line = point_str + " " + id_str + " " + cat_str + "\n";

            output_fs << line;
        }

        colormap_fs.close();
        output_fs.close();

        ROS_INFO("Color map saved succesfully to %s", color_map_file.c_str());
        ROS_INFO("Results saved succesfully to %s", txt_output_file.c_str());
    }
    else
    {
        ROS_INFO("Could not open file %s", txt_output_file.c_str());
    }

    if( config_.save_timings )
    {
        std::string timings_file = config_.output_name + "_timings.txt";

        double tracking_av = get_avg_time(tracking_times_);
        double preprocess_av = get_avg_time(preprocess_times_);
        double association_av = get_avg_time(association_times_);

        std::ofstream timings_fs;
        timings_fs.open(timings_file);
        if (timings_fs.is_open())
        {
            std::string line = "Fusion average: " + std::to_string(fusion_av) + "\n\n" +
                               "Tracking average: " + std::to_string(tracking_av) + "\n" +
                               "Preprocess average: " + std::to_string(preprocess_av) + "\n" +
                               "Association average: " + std::to_string(association_av) + "\n";

            timings_fs << line;

            ROS_INFO("Timings saved succesfully to %s", timings_file.c_str());
        }
        else
        {
            ROS_INFO("Could not open file %s", timings_file.c_str());
        }
        timings_fs.close();
    }

    return true;
}

void PanopticTracker::process_stuff_measurement(
    const unsigned int id,
    const unsigned int category,
    const Voxel point,
    const double weight)
{
    // NOT THREAD SAFE

    // for stuff, category == id
    fuse(point, category, category, weight);
}

void PanopticTracker::process_static_measurement(
    const unsigned int id,
    const unsigned int category,
    const Voxel point,
    const double weight)
{
    // NOT THREAD SAFE
    auto measurement_struct_it = measurements_.find(id);
    if( measurement_struct_it == measurements_.end() )
    {
        auto init_struct = std::make_shared<Object_struct>();
        measurements_[id] = init_struct;
        measurements_[id]->category = category;
    }
    measurements_[id]->points.insert(point);
    measurements_[id]->weights[point] = weight;
}

void PanopticTracker::process_dynamic_measurement(
    const unsigned int id,
    const unsigned int category,
    const Voxel point,
    const double weight)
{
    return;
}

/*
void PanopticTracker::outlier_rejection_callback(std::shared_ptr<Object_struct> measurement)
{
    if( config_.outlier_rejection_method == "dbscan" )
    {
        measurement->points = clustering_.reject_outliers(measurement->points);
    }
    else
    {
        Gaussian3D tmp_distribution;
        tmp_distribution.init(measurement->points.begin(), measurement->points.end());

        VoxelVector tmp_points;

        for( auto voxel : measurement->points )
        {
            Vector3 point = voxel.cast<float>();

            double sq_dist = Object_Tracker_Base::sq_mahalanobis(
                tmp_distribution, point);

            if( sq_dist < config_.outlier_threshold )
            {
                tmp_points.push_back(point.cast<voxblox::LongIndexElement>());
            }
        }

        measurement->points.clear();

        if( tmp_points.size() > config_.cluster_min_points )
        {
            measurement->points.insert(tmp_points.begin(), tmp_points.end());
        }
    }
}
*/

void PanopticTracker::outlier_rejection_callback(std::shared_ptr<Object_struct> measurement)
{
    if( !measurement->distribution.undef &&
        config_.outlier_rejection_method != "dbscan" )
    {
        measurement->points = outlier_rejection_->reject_outliers(
            measurement->points,
            measurement->distribution);
    }
    else
    {
        measurement->points = outlier_rejection_->reject_outliers(
            measurement->points);
    }
}

void PanopticTracker::sampling_callback(std::shared_ptr<Object_struct> measurement)
{
    VoxelVector samples;

    if( config_.max_samples > 0 && measurement->points.size() > 0 )
    {
        std::sample(
            measurement->points.begin(),
            measurement->points.end(),
            std::back_inserter(samples),
            (unsigned int)config_.max_samples,
            std::mt19937{std::random_device{}()});
    }
    else
    {
        return;
    }

    if( config_.sample_measurements )
    {
        for( auto s : samples )
        {
            measurement->samples.push_back(s.cast<float>());
        }
    }

    if( config_.infer_distribution )
    {
        measurement->distribution.init(samples.begin(), samples.end());
    }
}

void PanopticTracker::fuse_callback(
    std::shared_ptr<Object_struct> measurement,
    unsigned int id)
{
    if( measurement )
    {
        if( measurement->points.size() > 0)
        {
            for( auto point : measurement->points )
            {
                double weight = measurement->weights[point];
                fuse(point, id, measurement->category, weight);
            }
        }
    }
    else
    {
        ROS_WARN("Measurement is a nullptr");
    }

}

void PanopticTracker::fuse(
    Voxel point,
    unsigned int id,
    unsigned int category,
    double weight)
{
    if( ids->count(id) == 0 )
    {
        new_object(id);
    }
    (*ids)[id]->update_confidence(category);
    update_point(point, id, weight);
}

void PanopticTracker::new_object(unsigned int id)
{
    auto init = std::make_shared<Id_struct>();
    init->color =
        ( config_.visualise_global_ids &&
          !Common::val_found_in_id_arr(id, STUFF_CLASS_IDX, N_STUFF_CLASSES) )
        ? voxblox::randomColor() : CLASS_COLORS[id];

    (*ids)[id] = init;
}

void PanopticTracker::update_point(Voxel point, unsigned int id, double weight)
{
    if( map->count(point) == 0 )
    {
        (*map)[point] = std::make_pair(id, weight);
    }
    else
    {
        unsigned int old_id = (*map)[point].first;
        unsigned int old_weight = (*map)[point].second;

        if( id == old_id)
        {
            (*map)[point].second += weight;
        }
        else if( weight > old_weight )
        {
            (*map)[point].second = weight - old_weight;
            (*map)[point].first = id;
        }
        else
        {
            (*map)[point].second -= weight;
        }
    }
}

void PanopticTracker::save_time(std::chrono::duration<double> tdiff, std::vector<double>& vec)
{
    if( config_.save_timings )
    {
        vec.push_back(tdiff.count());
    }
}

double PanopticTracker::get_avg_time(std::vector<double>& times)
{
    double sum = 0;
    for(auto t : times)
    {
        sum += t;
    }
    return sum / times.size();
}
} // namespace fusion

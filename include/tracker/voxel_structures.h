/* raiviol 5/2021
*/

#ifndef FUSION_VOXEL_STRUCTURES_H
#define FUSION_VOXEL_STRUCTURES_H

#include <scannet_classes.h>
#include <distributions.h>

#include <stdint.h>
#include <memory>
#include <limits>

#include <set>
#include <unordered_set>
#include <vector>
#include <map>
#include <unordered_map>

#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/block_hash.h>

namespace fusion
{
    typedef std::array<float, N_CLASSES> confidence_array;

    typedef voxblox::LongIndexSet VoxelSet;
    typedef voxblox::LongIndexVector VoxelVector;
    typedef voxblox::LongIndexHashMapType<unsigned int>::type VoxelUintMap;

    struct id_struct
    {
        VoxelSet voxels;
        confidence_array confidence = {};
        confidence_array confidence_sum = {};
        unsigned int confidence_n = 0;

        uint8_t category = VOID_CLASS_ID;

        bool up_to_date = false;

        voxblox::Color color = VOID_COLOR;

        Gaussian3D voxel_distribution;

        void add_voxels(voxblox::GlobalIndex voxel_id)
        {
            voxels.insert(voxel_id);
            //up_to_date = false;
        }
        void add_voxels(VoxelSet& voxel_ids)
        {
            voxels.insert(voxel_ids.begin(), voxel_ids.end());
            //up_to_date = false;
        }

        void update_confidence(uint8_t cat_idx)
        { // assume cat_idx confidence == 1 -> others are 0
            if( cat_idx != VOID_CLASS_ID &&
                cat_idx < N_CLASSES )
            {
                confidence_sum[cat_idx] += 1;
                confidence_n += 1;

                up_to_date = false;
            }
        }
        void update_confidence(confidence_array& confidence)
        {
            for (uint8_t i=0; i<N_CLASSES; ++i)
            {
                confidence_sum[i] += confidence[i];
            }
            confidence_n += 1;

            up_to_date = false;
        }

        void update_category(double confidence_threshold)
        {
            if( !up_to_date )
            {
                category = VOID_CLASS_ID;

                double top_c = 0;

                if( confidence_n > 0 )
                {
                    for (uint8_t i=0; i<N_CLASSES; ++i)
                    {
                        confidence[i] = confidence_sum[i] / confidence_n;
                        if( confidence[i] > top_c )
                        {
                            top_c = confidence[i];
                            if( top_c > confidence_threshold )
                            {
                                category = i;
                            }
                        }
                    }
                }
                up_to_date = true;
            }
        }
        uint8_t get_category(double confidence_threshold)
        {
            update_category(confidence_threshold);
            return category;
        }
    };

    struct voxel_struct
    {
        double weight = 0;

        unsigned int global_id = VOID_CLASS_ID;
        voxblox::Color color = VOID_COLOR;

        void update_id(unsigned int new_global_id, double vx_weight=1)
        {
            if( new_global_id == global_id)
            {
                weight += vx_weight;
            }
            else if( vx_weight > weight )
            {
                weight = vx_weight - weight;
                global_id = new_global_id;
            }
            else
            {
                weight -= vx_weight;
            }
        }
    };
    struct compare_id_size
    {
        bool operator()(std::shared_ptr<id_struct> a, std::shared_ptr<id_struct> b)
        {
            return a->voxels.size() > b->voxels.size();
        }
    };

    typedef std::unordered_map<unsigned int, std::shared_ptr<id_struct>> IdMap;
    typedef voxblox::LongIndexHashMapType<std::shared_ptr<voxel_struct>>::type VoxelMap;

}; // end namespace fusion

#endif // FUSION_VOXEL_STRUCTURES_H

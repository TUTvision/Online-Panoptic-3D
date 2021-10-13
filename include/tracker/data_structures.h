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

typedef voxblox::GlobalIndex Voxel;
typedef voxblox::LongIndexSet VoxelSet;
typedef voxblox::LongIndexVector VoxelVector;
typedef voxblox::LongIndexHashMapType<unsigned int>::type VoxelUintMap;

typedef std::vector<Vector3> PointVector;

struct Object_struct
{
    unsigned int category;

    VoxelSet points;
    VoxelUintMap weights;

    Gaussian3D distribution;

    PointVector samples;
};

struct Association_struct
{
    unsigned int measurement_id;
    unsigned int target_id;

    double likelihood;
};

struct Id_struct
{
    double confidence = 0;
    confidence_array class_distribution;
    unsigned int category = VOID_CLASS_ID;
    voxblox::Color color = VOID_COLOR;

    bool up_to_date = false;

    unsigned int n;

    void update_confidence(unsigned int cat_idx, double weight=1)
    {
        class_distribution[cat_idx] += weight;

        ++n;
        up_to_date = false;
    }

    void update_category(double confidence_threshold)
    {
        if( !up_to_date )
        {
            double sum = 0;
            double top_c = 0;
            unsigned int top_i = VOID_CLASS_ID;

            if( n > 0 )
            {
                for( unsigned int i=0; i<N_CLASSES; ++i )
                {
                    double c = class_distribution[i];
                    sum += c;

                    if( c > top_c )
                    {
                        top_c = c;
                        top_i = i;
                    }
                }

                confidence = std::max(std::min(top_c / sum, 1.0), 0.0);

                if( top_c / sum > confidence_threshold)
                {
                    category = top_i;
                }
                else
                {
                    category = VOID_CLASS_ID;
                }
            }
            up_to_date = true;
        }
    }

    unsigned int get_category(double threshold)
    {
        update_category(threshold);
        return category;
    }
};

// <id : struct>
typedef std::unordered_map<unsigned int, std::shared_ptr<Id_struct>> IdMap;
typedef std::unordered_map<unsigned int, std::shared_ptr<Object_struct>> ObjectMap;

// <voxel coordinate, <id : weight>  >
typedef voxblox::LongIndexHashMapType<std::pair<unsigned int, double>>::type VoxelMap;

typedef std::vector<unsigned int> IdVector;
typedef std::unordered_set<unsigned int> IdSet;
typedef std::vector<Association_struct> AssociationVector;

}; // end namespace fusion

#endif // FUSION_VOXEL_STRUCTURES_H

/* raiviol 5/2021
*/

#ifndef FUSION_SCANNET_CLASSES_H
#define FUSION_SCANNET_CLASSES_H

#include <distributions.h>
#include <kalman_filtering.h>
#include <voxblox/core/color.h>

namespace fusion
{
    const unsigned int N_STUFF_CLASSES = 2;
    const unsigned int N_THING_CLASSES = 18;
    const unsigned int N_STATIC_CLASSES = 18;
    const unsigned int N_DYNAMIC_CLASSES = 0;

    const unsigned int N_CLASSES = N_THING_CLASSES + N_STUFF_CLASSES;

    const std::string STUFF_LABELS [N_STUFF_CLASSES] = {"wall", "floor"};
    const std::string THING_LABELS [N_THING_CLASSES] =
         {"cabinet", "bed", "chair", "sofa", "table", "door", "window",
          "bookshelf", "picture", "counter", "desk", "curtain", "refrigerator",
          "shower curtain", "toilet", "sink", "bathtub", "otherfurniture"};
    const std::string CLASS_LABELS [N_CLASSES] =
        {"wall", "floor", "cabinet", "bed", "chair", "sofa", "table", "door",
         "window", "bookshelf", "picture", "counter", "desk", "curtain", "refrigerator",
         "shower curtain", "toilet", "sink", "bathtub", "otherfurniture"};

    const unsigned int STUFF_CLASS_IDS [N_STUFF_CLASSES] = {1, 2};
    const unsigned int THING_CLASS_IDS [N_THING_CLASSES] =
        {3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 24, 28, 33, 34, 36, 39};
    const unsigned int VALID_CLASS_IDS [N_CLASSES] =
        {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 24, 28, 33, 34, 36, 39};

    const unsigned int STUFF_CLASS_IDX [N_STUFF_CLASSES] = {0, 1};
    const unsigned int THING_CLASS_IDX [N_THING_CLASSES] =
        {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};

    const unsigned int STATIC_OBJECT_IDX [N_STATIC_CLASSES] =
        {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
    const unsigned int DYNAMIC_OBJECT_IDX [N_DYNAMIC_CLASSES] = {};

    const unsigned int VOID_CLASS_ID = 255;
    const voxblox::Color VOID_COLOR = voxblox::Color(0,0,0);

    const voxblox::Color CLASS_COLORS [N_CLASSES] =
        {
            voxblox::Color(174, 199, 232), // wall
            voxblox::Color(152, 223, 138), // floor
            voxblox::Color(31, 119, 180),  // cabinet
            voxblox::Color(255, 187, 120), // bed
            voxblox::Color(188, 189, 34),  // chair
            voxblox::Color(140, 86, 75),   // sofa
            voxblox::Color(255, 152, 150), // table
            voxblox::Color(214, 39, 40),   // door
            voxblox::Color(197, 176, 213), // window
            voxblox::Color(148, 103, 189), // bookshelf
            voxblox::Color(196, 156, 148), // picture
            voxblox::Color(23, 190, 207),  // counter
            voxblox::Color(247, 182, 210), // desk
            voxblox::Color(219, 219, 141), // curtain
            voxblox::Color(255, 127, 14),  // refrigerator
            voxblox::Color(158, 218, 229), //  shower curtain
            voxblox::Color(44, 160, 44),   // toilet
            voxblox::Color(112, 128, 144), //  sink
            voxblox::Color(227, 119, 194), //  bathtub
            voxblox::Color(82, 84, 163),   //  otherfurn
        };
}; // namespace fusion

#endif // FUSION_SCANNET_CLASSES_H

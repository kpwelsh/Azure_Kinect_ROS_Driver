
#ifndef K4A_ROS_IMAGE_FILTER_H
#define K4A_ROS_IMAGE_FILTER_H
#include <k4a/k4a.h>
#include <k4a/k4a.hpp>

namespace K4AImageFilter {
    void sharpFilter(k4a::image & img, float strength);
}

#endif  // K4A_ROS_IMAGE_FILTER_H

#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
#include "azure_kinect_ros_driver/k4a_image_filter.h"
#include <cmath>
#include <iostream>

using namespace std;


void getXY(const k4a::image & img, const int & i, int & x, int & y) {
    x = i % img.get_width_pixels();
    y = i / img.get_width_pixels();
}

void getI(const k4a::image & img, int & i, const int x, const int y) {
    i = x;
    i += y * img.get_width_pixels();
    if (i < 0) i = -1;
    else if (i >= img.get_size()) i = -1;
}

void K4AImageFilter::sharpFilter(k4a::image & img, float strength) {
    int w = img.get_width_pixels();
    int h = img.get_height_pixels();
    int x, y, j;
    float nextValue;
    int z_index;
    auto buffer = img.get_buffer();
    uint8_t * nextBuffer = new uint8_t[img.get_size()];

    for (size_t i = 0; i < w * h; i++)
    {
        getXY(img, i, x, y);
        nextValue = buffer[2 * i] + 4 * strength * buffer[2 * i];
        getI(img, j, x - 1, y);
        if (j >= 0) nextValue += -1 * strength * buffer[2 * j];
        getI(img, j, x + 1, y);
        if (j >= 0) nextValue += -1 * strength * buffer[2 * j];
        getI(img, j, x, y - 1);
        if (j >= 0) nextValue += -1 * strength * buffer[2 * j];
        getI(img, j, x, y + 1);
        if (j >= 0) nextValue += -1 * strength * buffer[2 * j];
        if (nextValue < 0) continue;//nextValue = 0;
        if (nextValue > 254) continue;//nextValue = 254;
        nextBuffer[i] = (uint8_t) nextValue;
    }

    for (size_t i = 0; i < w * h; i++)
    {
        buffer[2 * i] = nextBuffer[i];
    }

    delete[] nextBuffer;
}
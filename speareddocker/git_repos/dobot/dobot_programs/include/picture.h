//
// Created by lars on 14.02.19.
//

#ifndef PROJECT_PICTURE_H
#define PROJECT_PICTURE_H

#include <fstream>
#include <vector>
#include <cstdint>
#include <cstdlib>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>

/**
 * enumeration of all supported colors of blocks
 */
enum PictureColor {
    RED, BLUE, GREEN, YELLOW, RED_SET, BLUE_SET, GREEN_SET, YELLOW_SET, UNKNOWN
};

/**
 * basic class handling the picture information used by the dobot to draw the picture.
 */
class Picture {
private:
    //picture information stored in row major order
    std::vector<PictureColor> picture;
    size_t rows, cols;
    double rotation;

    void find_next_block(const PictureColor color, size_t& row, size_t& col) const;

public:
    Picture(size_t const width, const size_t height);
    size_t get_rows() const;
    size_t get_cols() const;
    bool set_color(const size_t x, const size_t y, const uint8_t h, const uint8_t s, const uint8_t v);
    PictureColor get_color(const size_t x, const size_t y) const;
    bool is_color_needed(const PictureColor color) const;
    bool set_color_set(const size_t x, const size_t y);
    bool unset_color_set(const size_t x, const size_t y);
    void reset_color_set();
    tf2::Vector3 get_next_set_position(const tf2::Vector3 &start, const PictureColor c, const tf2::Vector3 &block_size,
                                    size_t &row, size_t &col);
    void ostream_show(std::ostream &out) const;
};

/**
 * picture namespace
 */
namespace pic
{
    PictureColor convert_rgb_to_pic_color(const uint8_t r, const uint8_t g, const uint8_t b);
    PictureColor convert_hsv_to_pic_color(const uint8_t h, const uint8_t s, const uint8_t v);
}

#endif //PROJECT_PICTURE_H

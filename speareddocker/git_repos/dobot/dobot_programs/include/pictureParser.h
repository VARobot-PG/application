//
// Created by lars on 05.02.19.
//

#ifndef PROJECT_PICTUREPARSER_H
#define PROJECT_PICTUREPARSER_H

#include <ros/ros.h>
#include <string>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "picture.h"

/**
 * This class handles all the picture parsing and dobot picture generation.
 */
class PictureParser {
private:
    std::unique_ptr<cv::Mat> rt_img;

    void hsv_color(const size_t xcell, const size_t ycell,
                   const cv::Mat &rszd_img, uint8_t &h, uint8_t &s, uint8_t &v) const;
public:
    PictureParser(const std::string img_path);
    void load_image(const std::string img_path);
    long get_width() const;
    long get_height() const;
    Picture generate_dobot_picture(const size_t width, const size_t height) const;
};

#endif //PROJECT_PICTUREPARSER_H

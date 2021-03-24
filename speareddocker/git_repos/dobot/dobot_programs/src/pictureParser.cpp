//
// Created by lars on 05.02.19.
//

#include "../include/pictureParser.h"

/**
 * computes the hsv color of a pixel.
 * @param xcell column of the pixel
 * @param ycell row of the pixel
 * @param rszd_img the image containing the pixel
 * @param h hue value
 * @param s saturation value
 * @param v value value
 */
void PictureParser::hsv_color(const size_t xcell, const size_t ycell,
                              const cv::Mat &rszd_img, uint8_t &h, uint8_t &s, uint8_t &v) const
{
    if(xcell >= rszd_img.cols || ycell >= rszd_img.rows)
        ROS_ERROR("Pixel does not exist at coordinate (%lu,%lu)!", xcell, ycell);

    //get pixel
    cv::Vec3b px = rszd_img.at<cv::Vec3b>(cv::Point(xcell, ycell));

    //get pixel color
    h = px.val[0];
    s = px.val[1];
    v = px.val[2];
}


/**
 * default constructor, loads the image into main memory
 * @param img_path path to the image to parse
 */
PictureParser::PictureParser(const std::string img_path)
{
    load_image(img_path);
}

/**
 * loads the image into main memory
 * @param img_path path to the image
 */
void PictureParser::load_image(const std::string img_path)
{
    rt_img = std::make_unique<cv::Mat>(cv::imread(img_path.c_str(), cv::IMREAD_COLOR));
}


/**
 * returns the height of the image
 * @return height of the handled image
 */
long PictureParser::get_height() const
{
    if(!rt_img)
        ROS_ERROR("No image has been loaded!\n");

    return rt_img->rows;
}

/**
 * returns the width of the handled image
 * @return width of the handled image
 */
long PictureParser::get_width() const
{
    if(!rt_img)
        ROS_ERROR("No image has been loaded!\n");

    return rt_img->cols;
}

/**
 * computes the picture for the dobot drawing program from the handled picture
 * @param width desired width of the dobot picture
 * @param height desired height of the dobot picture
 * @return dobot picture
 */
Picture PictureParser::generate_dobot_picture(const size_t width, const size_t height) const
{
    Picture p(width, height);

    cv::Size sz(width, height);
    static cv::Mat resized;

    //scale image to dobot working size
    cv::resize(*rt_img, resized, sz);

    //get hsv image for better color handling
    cv::cvtColor(resized, resized, cv::COLOR_BGR2HSV);

    uint8_t h,s,v;

    for(size_t i=0; i<height; i++)
    {
        for(size_t j=0; j<width; j++)
        {
            hsv_color(j, i, resized, h, s, v);
            p.set_color(j, i, h, s, v);
        }
    }

    return p;
}
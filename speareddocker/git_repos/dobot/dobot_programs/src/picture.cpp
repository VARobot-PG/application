//
// Created by lars on 14.02.19.
//

#include "../include/picture.h"
#include <dobot/dobot.h>

/**
 * converts a color in rgb format of the color sensor to the corresponding block color.
 * For unsupported colors returns PictureColor::UNKNOWN
 * @param r red value
 * @param g green value
 * @param b blue value
 * @return corresponding PictureColor or PictureColor::UNKNOWN
 */
PictureColor pic::convert_rgb_to_pic_color(const uint8_t r, const uint8_t g, const uint8_t b)
{
    switch(dobot::convert_rgb_to_block_color(r, g, b))
    {
        case dobot::BlockColor::RED:
            return PictureColor::RED;

        case dobot::BlockColor::GREEN:
            return PictureColor::GREEN;

        case dobot::BlockColor::BLUE:
            return PictureColor::BLUE;

        case dobot::BlockColor::YELLOW:
            return PictureColor::YELLOW;

        case dobot::BlockColor::UNKNOWN:
            return PictureColor::UNKNOWN;

        default:
            return PictureColor::UNKNOWN;
    }
}


/**
 * converts a color in hsv format to corresponding block color.
 * @param h hue value
 * @param s saturation value
 * @param v value value
 * @return corresponding PictureColor
 */
PictureColor pic::convert_hsv_to_pic_color(const uint8_t h, const uint8_t s, const uint8_t v)
{
    //h has a maximum of 179 and is therefore just half of the circle degree
    uint16_t deg = h * 2;

    //based on hsv color circle
    if(deg >= 0 && deg < 40)
        return PictureColor::RED;
    if(deg >= 40 && deg < 80)
        return PictureColor::YELLOW;
    if(deg >= 80 && deg < 170)
        return PictureColor::GREEN;
    if(deg >= 170 && deg < 280)
        return PictureColor::BLUE;

    //280-360
    return PictureColor::RED;
}

/**
 * default constructor for a picture
 * @param width the width of the picture in pixels (blocks)
 * @param height the height of the picture in pixels (blocks)
 */
Picture::Picture(const size_t width, const size_t height)
{
    this->rows = height;
    this->cols = width;
    this->rotation = 0;
    picture.resize(get_rows()*get_cols());
}


/**
 * returns the number of columns (width) of this picture.
 * @return width of this picture
 */
size_t Picture::get_cols() const
{
    return cols;
}


/**
 * returns the number of rows (height) of this picture.
 * @return height of this picture
 */
size_t Picture::get_rows() const
{
    return rows;
}


/**
 * sets the color of the pixel (x,y) to the corresponding block color.
 * @param x column of the pixel
 * @param y row of the pixel
 * @param h hue value
 * @param s saturation value
 * @param v value value
 * @return true iff the pixel exists
 */
bool Picture::set_color(const size_t x, const size_t y, const uint8_t h, const uint8_t s, const uint8_t v)
{
    if(x >= get_cols() || y >= get_rows())
        return false;

    picture[y*get_cols()+x] = pic::convert_hsv_to_pic_color(h, s, v);

    return true;
}

/**
 * returns the color of the pixel at position (x,y)
 * @param x column of the pixel
 * @param y row of the pixel
 * @return block color of the pixel
 */
PictureColor Picture::get_color(const size_t x, const size_t y) const
{
    if(x >= get_cols() || y >= get_rows())
        ROS_ERROR("Invalid row or col number!");

    return picture[y*get_cols()+x];
}
/**
 * returns true iff there exists a pixel in this picture with the corresponding color that is not yet drawn.
 * @param c
 * @return true iff there is a pixel with specified color that has yet to be drawn
 */
bool Picture::is_color_needed(const PictureColor color) const
{
    for(size_t r=0; r<get_rows(); r++)
    {
        for(size_t c=0; c<get_cols(); c++)
        {
            if(picture[r*get_cols()+c] == color)
                return true;
        }
    }
    
    return false;
}

/**
 * sets the color of the given pixel to set. Marks pixel as drawn.
 * @param x column of the pixel
 * @param y row of the pixel
 * @return true iff pixel exists and has not already been marked
 */
bool Picture::set_color_set(const size_t x, const size_t y)
{
    if(x>= get_cols() || y >= get_rows())
        return false;

    size_t elem = y*get_cols() + x;

    switch(picture[elem])
    {
        case PictureColor::RED:
            picture[elem] = PictureColor::RED_SET;
            break;
        case PictureColor::BLUE:
            picture[elem] = PictureColor::BLUE_SET;
            break;
        case PictureColor::GREEN:
            picture[elem] = PictureColor::GREEN_SET;
            break;
        case PictureColor::YELLOW:
            picture[elem] = PictureColor::YELLOW_SET;
            break;
        default:
            return false;
    }

    return true;
}

/**
 * sets color of the given pixel to unset. Marks pixel as not drawn.
 * By default every cell is not drawn until set_color_set for the pixel  is called.
 * @param x column of the pixel
 * @param y row of the pixel
 * @return true iff pixel exists and color could be unset
 */
bool Picture::unset_color_set(const size_t x, const size_t y)
{
    if(x>= get_cols() || y >= get_rows())
        return false;

    size_t elem = y*get_cols() + x;

    switch(picture[elem])
    {
        case PictureColor::RED_SET:
            picture[elem] = PictureColor::RED;
            break;
        case PictureColor::BLUE_SET:
            picture[elem] = PictureColor::BLUE;
            break;
        case PictureColor::GREEN_SET:
            picture[elem] = PictureColor::GREEN;
            break;
        case PictureColor::YELLOW_SET:
            picture[elem] = PictureColor::YELLOW;
            break;
        default:
            return false;
    }

    return true;
}

/**
 * resets draw status of picture to completely undrawn.
 */
void Picture::reset_color_set()
{
    for(size_t r=0; r<get_rows(); r++)
    {
        for(size_t c=0; c<get_cols(); c++)
        {
            unset_color_set(c, r);
        }
    }
}

/**
 * returns the position where to place the next block of color c
 * @param start the point where to start from (upper left corner)
 * @param c the color of the block to draw
 * @param block_size size of one block
 * @param row the row where the block is
 * @param col the column where the block is
 * @return position of the block to draw
 */
tf2::Vector3 Picture::get_next_set_position(const tf2::Vector3 &start, const PictureColor c, const tf2::Vector3 &block_size,
                                        size_t &row /*out*/, size_t &col /*out*/)
{
    find_next_block(c, row, col);
    int z = 0;

    tf2::Vector3 offset = tf2::Vector3(-((int)row),-((int)col),z); //Offset measured in number of blocks
    offset = offset * block_size;							 //Convert offset from number of blocks to millimeter

    //ROTATION:
    tf2::Vector3 offsetRotated;
    offsetRotated.setX((offset.x() * cos(rotation)) -(offset.y() * sin(rotation)));
    offsetRotated.setY((offset.x() * sin(rotation)) +(offset.y() * cos(rotation)));
    offsetRotated.setZ(offset.z());
    ROS_INFO("Offset: [%f, %f, %f]",offsetRotated.x(),offsetRotated.y(),offsetRotated.z());

    return (start + offsetRotated);						 //Convert offset to LOAD_POSITION coordinates
}


/**
 * finds the next block with the specified color and stores the position into row and col.
 * @param color the color of the block to find
 * @param row the row where the block was found
 * @param col the col where the block was found
 */
void Picture::find_next_block(const PictureColor color, size_t &row, size_t &col) const
{
    for(size_t r=0; r<get_rows(); r++)
    {
        for(size_t c=0; c<get_cols(); c++)
        {
            if(picture[r*get_cols()+c] == color)
            {
                row = r;
                col = c;
                return;
            }
        }
    }
}


/**
 * shows the picture represented by this object on the console
 */
void Picture::ostream_show(std::ostream &out) const
{
    for(size_t r = 0; r < get_rows(); r++)
    {
        for(size_t c = 0; c < get_cols(); c++)
        {
            switch(picture[r*get_cols()+c])
            {
                case PictureColor::RED:
                    out << "R\t";
                    break;
                case PictureColor::GREEN:
                    out << "G\t";
                    break;
                case PictureColor::BLUE:
                    out << "B\t";
                    break;
                case PictureColor::YELLOW:
                    out << "Y\t";
                    break;
            }
        }

        out << std::endl;
    }
}

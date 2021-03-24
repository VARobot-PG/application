//
// Created by lars on 26.08.19.
//

#include "../include/domino_field.h"

namespace domino
{
    std::tuple<size_t, size_t> computeRotPos(const std::tuple<size_t, size_t> pos, const Orientation o)
    {
        switch(o)
        {
            case RIGHT:
                return std::make_tuple(std::get<0>(pos)+1, std::get<1>(pos));
            case BOTTOM:
                return std::make_tuple(std::get<0>(pos), std::get<1>(pos)-1);
            case LEFT:
                return std::make_tuple(std::get<0>(pos)-1, std::get<1>(pos));
            case TOP:
                return std::make_tuple(std::get<0>(pos), std::get<1>(pos)+1);
        }
    }

    Position getPosition(const size_t pos)
    {
        switch (pos)
        {
            case 0:
                return LEFT_END;
            case 1:
                return RIGHT_END;
        }
    }

    Orientation getOrientation(const size_t orient)
    {
        switch (orient)
        {
            case 0:
                return RIGHT;
            case 1:
                return BOTTOM;
            case 2:
                return LEFT;
            case 3:
                return TOP;
        }
    }

   DominoField::DominoField()
   {
       //field has 16x13 elems
       field = std::vector<int>(FIELD_HEIGHT*FIELD_WIDTH, -1);
   }

   size_t DominoField::convertPosition(const std::tuple<size_t, size_t> pos) const {
       return std::get<0>(pos)+std::get<1>(pos)*FIELD_WIDTH;
   }

   bool DominoField::computeConnection(const std::tuple<size_t, size_t> stone, const domino::Position position) const
   {
        size_t connectValue = position == LEFT_END ? getLeftEndValue() : getRightEndValue();
        return std::get<0>(stone) == connectValue;
   }

   bool DominoField::checkMatchingStones(const std::tuple<size_t, size_t> stone, const size_t value, const bool connectFirst) const
   {
       size_t stone_number = connectFirst ? std::get<0>(stone) : std::get<1>(stone);
       return stone_number == value;
   }

   bool DominoField::checkBounds(const std::tuple<size_t, size_t> conn_pos, const std::tuple<size_t, size_t> end_pos) const
   {
       return std::get<0>(conn_pos) < FIELD_WIDTH && std::get<1>(conn_pos) < FIELD_HEIGHT
           && std::get<0>(end_pos) < FIELD_WIDTH && std::get<1>(end_pos) < FIELD_HEIGHT
           && field[convertPosition(conn_pos)] == -1
           && field[convertPosition(end_pos)] == -1;
   }

   void DominoField::placeInitialStone(const std::tuple<size_t, size_t> stone)
   {
       size_t pos = convertPosition(MID_POSITION);
       //place stone and set end values
       left_value = field[pos] = std::get<0>(stone);
       right_value = field[pos+1] = std::get<1>(stone);
       //set next place positions that mark the end of the line
       left_end = std::make_tuple(std::get<0>(MID_POSITION)-1, std::get<1>(MID_POSITION));
       right_end = std::make_tuple(std::get<0>(MID_POSITION)+2, std::get<1>(MID_POSITION));
   }

   bool DominoField::placeStone(const std::tuple<size_t, size_t> stone, const domino::Position p, const domino::Orientation o)
   {
       auto connect_pos = p == LEFT_END ? left_end : right_end;
       auto end_pos = computeRotPos(connect_pos, o);
       bool connectLeft = computeConnection(stone, p);

       if(!checkMatchingStones(stone, p == LEFT_END ? left_value : right_value, connectLeft)
            || !checkBounds(connect_pos, end_pos))
           return false;

       //set new last insert positions
       last_connect_pos = connect_pos;
       last_end_point_pos = end_pos;

       //determine connect value and end value
       size_t connect_value = connectLeft ? std::get<0>(stone) : std::get<1>(stone);
       size_t end_value = connectLeft ? std::get<1>(stone) : std::get<0>(stone);

       //place stone
       field[convertPosition(connect_pos)] = connect_value;
       field[convertPosition(end_pos)] = end_value;

       //update ends
       if(p == RIGHT_END)
       {
           right_value = end_value;
           right_end = computeRotPos(end_pos, o);
       }
       else
       {
           left_value = end_value;
           left_end = computeRotPos(end_pos, o);
       }

       return true;
   }

   size_t DominoField::getLeftEndValue() const {
        return left_value;
    }

    size_t DominoField::getRightEndValue() const {
        return right_value;
    }

   void DominoField::getLastInsertPosition(std::tuple<size_t, size_t> &connectedSide,
                                           std::tuple<size_t, size_t> &endSide) const
   {
        connectedSide = last_connect_pos;
        endSide = last_end_point_pos;
   }
}


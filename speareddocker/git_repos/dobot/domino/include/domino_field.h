//
// Created by lars on 26.08.19.
//

#ifndef SRC_DOMINOFIELD_H
#define SRC_DOMINOFIELD_H

#include <tuple>
#include <vector>

namespace domino
{
    enum Position { LEFT_END, RIGHT_END };

    const std::tuple<size_t, size_t> MID_POSITION = std::make_tuple(7,6);

    /**
     * RIGHT: 1|2
     * BOTTOM: 1
     *         2
     * LEFT: 2|1
     * TOP: 2
     *      1
     */
    enum Orientation { RIGHT, BOTTOM, LEFT, TOP};

    /**
     * Returns the position based on a size_t input.
     * 0 = LEFT_END
     * 1 = RIGHT_END
     *
     * @param pos
     * @return
     */
    Position getPosition(const size_t pos);

    /**
     * Returns the orientation based on a size_t input.
     * 0 = RIGHT
     * 1 = BOTTOM
     * 2 = LEFT
     * 3 = TOP
     *
     * @param orient
     * @return
     */
    Orientation getOrientation(const size_t orient);

    /**
     * Field looks like this:
     * 12
     * 11
     * 10
     * .
     * 0
     *  1 2 3 4 ... 16
     */
    class DominoField {
    private:
        std::vector<int> field;
        std::tuple<size_t , size_t> left_end, right_end, last_connect_pos, last_end_point_pos;
        size_t left_value, right_value;
        const size_t FIELD_WIDTH = 16, FIELD_HEIGHT = 13;

        /**
         * converts a two dimensional position into a one dimensional position for inserting into the field vector
         *
         * @param pos the two dimensional position to convert
         * @return one dimensional position equivalent to two dimensional position
         */
        size_t convertPosition(const std::tuple<size_t, size_t> pos) const;
        /**
         * Checks if the stone to place has the same number as the stone already laying on the field.
         *
         * @param stone the stone to place
         * @param value the value of the line end to connect to
         * @param connectFirst whether the given stone should be connected to with its first or second number
         * @return
         */
        bool checkMatchingStones(const std::tuple<size_t, size_t> stone, const size_t value, const bool connectFirst) const;
        /**
         * Checks if the given stone can be placed at the given position.
         *
         * @param conn_pos the position where to place the stone to connect to the domino line
         * @param end_pos the new end position of the line
         * @return true iff the stone can be placed
         */
        bool checkBounds(const std::tuple<size_t, size_t> conn_pos, const std::tuple<size_t, size_t> end_pos) const;
        /**
         * Computes which side needs to be connected to the the endpoint.
         * @param stone the stone to be placed
         * @param position the position where the stone should be placed
         * @return {@code true} iff the left side of the stone must be connected
         */
        bool computeConnection(const std::tuple<size_t, size_t> stone, const Position position) const;
    public:
        DominoField();
        /**
         * Places the given inital stone onto the domino field
         * @param stone the stone to place
         */
        void placeInitialStone(const std::tuple<size_t, size_t> stone);
        /**
         * Places the given stone at the given position in the given rotation
         * @param stone the stone to place
         * @param p the position where to place the stone
         * @param o the orientation in which to place the stone
         * @return true iff stone can be placed at the given position in the given rotation
         */
        bool placeStone(const std::tuple<size_t, size_t> stone, const Position p, const Orientation o);
        /**
         * Retrieves the grid position of the last inserted stone.
         *
         * @param connectedSide connected side of the stone
         * @param endSide end point side of the stone
         */
        void getLastInsertPosition(std::tuple<size_t, size_t>& connectedSide, std::tuple<size_t, size_t>& endSide) const;
        /**
         * @return returns the value of the left most stone
         */
        size_t getLeftEndValue() const;
        /**
         * @return returns the value of the right most stone
         */
        size_t getRightEndValue() const;
    };
}

#endif //SRC_DOMINOFIELD_H

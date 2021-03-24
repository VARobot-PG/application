//
// Created by lars on 26.08.19.
//

#ifndef SRC_DOMINOPLAYER_H
#define SRC_DOMINOPLAYER_H

#include <ros/ros.h>
#include <tuple>
#include <vector>
#include <memory>
#include <iostream>
#include <algorithm>
#include <dobot/dobot.h>

#include "domino_field.h"
#include "domino_pool.h"

namespace domino
{
    enum Player { AI_PLAYER, HUMAN_PLAYER };

	const int WIN_POINTS = 5;
	const size_t MAX_TRIES = 3;

    class DominoPlayer {
	private:
		std::vector<std::tuple<size_t, size_t>> stones;
    protected:
        DominoPlayer();
        /**
		 * Checks if the player is still able to lay at least one stone.
		 *
		 * @param field the current state of the field
		 * @return {@code true} iff the player is able to lay at least one stone from his hand
		 */
        bool canLayStone(const DominoField& field) const;
    public:
		/**
		*	performs the turn of the player
		*/
        virtual bool turn(DominoField& field, DominoStonePool& stone_pool) = 0;
		/**
		*	returns the number of stones the player currently possesses
		*/
        size_t getNumberOfStones() const;
		/**
		*	counts how many points the player will get or lose at the end of a round
		*/
        int count() const;
		/**
		*	initializes the player's stones
		*/
        void initStones(std::vector<std::tuple<size_t, size_t>> stones);
		/**
		*	adds a drawn stone to the players hand
		*/
		void addStone(std::tuple<size_t, size_t> stone);
		/**
		*	Checks if player has this exact stone. Returns false if no such stone exists
		*	in the player's hand. left <= right always has to hold.
		*/
		bool hasStone(std::tuple<size_t, size_t> stone) const;
		/**
		 * Removes the given stone from the player's hand.
		 * @param stone the stone to remove. Note: left <= right
		 */
		void removeStone(std::tuple<size_t, size_t> stone);
    };

    class AIDominoPlayer : public DominoPlayer {
    private:
        dobot::Dobot arm;
        /**
         * Draws a stone from the stone_pool.
         * @return the drawn stone
         */
        std::tuple<size_t, size_t> drawStone(DominoStonePool stone_pool);
        /**
         * Selects a stone and decides where to place it.
         *
         * @param field the current state of the field
         * @return a stone and a posiition to place
         */
        std::tuple<std::tuple<size_t, size_t>, Position> selectStoneAndPosition(DominoField field);
        /**
         * selects the orientation of the stone to place.
         *
         * @param field the current state of the field
         * @param stone the stone to place
         * @param position where to place the stone
         * @return the orientation to place the stone in
         */
        Orientation selectPlacement(DominoField field, std::tuple<size_t, size_t> stone, Position position);
        /**
         * Places the given stone at the given location in the correct orientation.
         * @param stone the stone to place
         * @param connectSide the grid position of the connected side
         * @param endSide the gird position of the disconnected side
         */
        void placeStone(std::tuple<size_t, size_t> stone, std::tuple<size_t, size_t> connectSide,
                        std::tuple<size_t, size_t> endSide);
    public:
        AIDominoPlayer(std::shared_ptr<ros::NodeHandle> nh_ptr);
        bool turn(DominoField& field, DominoStonePool& stone_pool) override;
    };

    class HumanDominoPlayer : public DominoPlayer {
    public:
        bool turn(DominoField& field, DominoStonePool& stone_pool) override;
    };

    std::shared_ptr<DominoPlayer> resolvePlayer(Player p, std::shared_ptr<ros::NodeHandle> nh_ptr);
}

#endif //SRC_DOMINOPLAYER_H

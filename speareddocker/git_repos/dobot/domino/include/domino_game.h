//
// Created by lars on 26.08.19.
//

#ifndef SRC_DOMINO_GAME_H
#define SRC_DOMINO_GAME_H

#include <ros/ros.h>
#include "domino_player.h"
#include "domino_field.h"
#include "domino_pool.h"
#include <vector>
#include <random>
#include <iostream>
#include <memory>

namespace domino
{
    const std::vector<std::tuple<size_t,size_t>> DOMINO_DECK {
              std::make_tuple(0,0), std::make_tuple(0,1), std::make_tuple(0,2), std::make_tuple(0,3),
              std::make_tuple(0,4), std::make_tuple(0,5), std::make_tuple(0,6),
              std::make_tuple(1,1), std::make_tuple(1,2), std::make_tuple(1,3), std::make_tuple(1,4),
              std::make_tuple(1,5), std::make_tuple(1,6),
              std::make_tuple(2,2), std::make_tuple(2,3), std::make_tuple(2,4), std::make_tuple(2,5),
              std::make_tuple(2,6),
              std::make_tuple(3,3), std::make_tuple(3,4), std::make_tuple(3,5), std::make_tuple(3,6),
              std::make_tuple(4,4), std::make_tuple(4,5), std::make_tuple(4,6),
              std::make_tuple(5,5), std::make_tuple(5,6),
              std::make_tuple(6,6)
    };

    class DominoGame
    {
    private:
        bool player_1_turn;
        std::shared_ptr<DominoPlayer> p1, p2;
        DominoField field;
        DominoStonePool stone_pool;
    public:
		/**
		*	creates a new game instance with the given players
		*/
        DominoGame(Player one, Player two, std::shared_ptr<ros::NodeHandle> nh_ptr);
		/**
		*	sets up the initial game state
		*/
        void setupGame();
		/**
		*	starts the game
		*/
        bool play();
    };
}

#endif //SRC_DOMINO_GAME_H

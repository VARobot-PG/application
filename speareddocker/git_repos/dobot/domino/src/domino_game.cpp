//
// Created by lars on 26.08.19.
//

#include "../include/domino_game.h"

namespace domino
{
    DominoGame::DominoGame(domino::Player one, domino::Player two, std::shared_ptr<ros::NodeHandle> nh_ptr)
            : stone_pool(DOMINO_DECK), p1(resolvePlayer(one, nh_ptr)), p2(resolvePlayer(two, nh_ptr)) {}

    void DominoGame::setupGame()
    {
        field.placeInitialStone(stone_pool.draw());

        std::default_random_engine generator;
        std::uniform_int_distribution<int> distribution = std::uniform_int_distribution<int>(0,1);
        player_1_turn = distribution(generator) == 1;

        //give players their stones for testing
        //TODO: in a real game this has to be entered manually or detected by the camera
        std::vector<std::tuple<size_t, size_t>> stones_1, stones_2;
        std::tie(stones_1, stones_2) = stone_pool.drawInitStones();
        p1->initStones(stones_1);
        p2->initStones(stones_2);
    }

    bool DominoGame::play()
    {
        bool no_turn_possible = false;

        //one player has no more stones or there are only 2 stones left to draw but the last player to do his turn could not do it
        while(p1->getNumberOfStones() > 0 && p2->getNumberOfStones() > 0
           && !(stone_pool.size() == 2 && !no_turn_possible))
        {
            std::shared_ptr<DominoPlayer> player = player_1_turn ? p1 : p2;
            if(!player->turn(field, stone_pool))
                no_turn_possible = true;
            player_1_turn = !player_1_turn;
        }

        if(p1->count() == p2->count())
        {
            std::cout << "Draw! Neither of the players wins!" << std::endl;
        }

        int winner, loser, winner_points, loser_points;

        if(p1->count() > p2->count())
        {
            winner = 1, winner_points = p1->count();
            loser = 2, loser_points = p2->count();
        }
        else
        {
            winner = 2, winner_points = p2->count();
            loser = 1, loser_points = p1->count();
        }

        std::cout << "Player " << winner << " has won against player " << loser << " with " << winner_points << " to "
                  << loser_points << " points!" << std::endl;

        return true;
    }
}
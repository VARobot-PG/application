//
// Created by lars on 26.08.19.
//
#include <ros/ros.h>
#include <memory>

#include "../include/domino_game.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Domino");
    std::shared_ptr<ros::NodeHandle> nh_ptr = std::make_shared<ros::NodeHandle>();
    domino::DominoGame game(domino::Player::HUMAN_PLAYER, domino::Player::HUMAN_PLAYER, nh_ptr);

    game.setupGame();
    game.play();

    return 0;
}
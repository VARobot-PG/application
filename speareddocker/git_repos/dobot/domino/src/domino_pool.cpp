//
// Created by lars on 26.08.19.
//

#include "../include/domino_pool.h"

namespace domino
{
    DominoStonePool::DominoStonePool(std::vector<std::tuple<size_t, size_t>> init_stone_pool)
    {
        stone_pool = init_stone_pool;
    }

    DominoStonePool::DominoStonePool(domino::DominoStonePool &d)
    {
        stone_pool = std::vector<std::tuple<size_t, size_t>>(d.stone_pool);
        generator = std::default_random_engine();
    }

    size_t DominoStonePool::size() { return stone_pool.size(); }

    std::tuple<size_t, size_t> DominoStonePool::draw()
    {
        std::uniform_int_distribution<int> distribution(0, stone_pool.size()-1);
        int index = distribution(generator);
        auto stone = stone_pool[index];
        stone_pool.erase(stone_pool.begin()+index);

        return stone;
    }

    std::tuple< std::vector<std::tuple<size_t, size_t>>, std::vector<std::tuple<size_t, size_t>>> DominoStonePool::drawInitStones()
    {
        std::vector<std::tuple<size_t, size_t>> stones_1, stones_2;

        for(int i=0; i<7; i++)
        {
            stones_1.push_back(draw());
            stones_2.push_back(draw());
        }

        return std::make_tuple(stones_1, stones_2);
    }

    bool DominoStonePool::removeStone(std::tuple<size_t, size_t> toRemove)
    {
        if(std::find(stone_pool.begin(), stone_pool.end(), toRemove)  != stone_pool.end())
        {
            stone_pool.erase(std::remove(stone_pool.begin(), stone_pool.end(), toRemove), stone_pool.end());
            return true;
        }

        return false;
    }
}


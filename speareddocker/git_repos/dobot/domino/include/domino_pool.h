//
// Created by lars on 26.08.19.
//

#ifndef SRC_DOMINO_POOL_H
#define SRC_DOMINO_POOL_H

#include <tuple>
#include <vector>
#include <random>
#include <algorithm>

namespace domino
{
    class DominoStonePool
    {
    private:
        std::vector<std::tuple<size_t, size_t>> stone_pool;
        std::default_random_engine generator;
    public:
        /**
         * Initializes the stone pool with stones.
         * @param init_stone_pool the stones to be used for initialization
         */
        DominoStonePool(std::vector<std::tuple<size_t, size_t>> init_stone_pool);
        /**
         * Copy constructor
         */
        DominoStonePool(DominoStonePool &d);
        /**
         * Draws a stone from the pool.
         *
         * @return the stone that was drawn
         */
        std::tuple<size_t, size_t> draw();
        /**
         * Draws the intial stones for both players.
         * @return a tuple containing both player's initial stones
         */
        std::tuple< std::vector<std::tuple<size_t, size_t>>,  std::vector<std::tuple<size_t, size_t>>> drawInitStones();
        /**
         * Deletes the given stone from the stone pool.
         *
         * @param toRemove the stone to remove
         * @return {@link true} iff stone has been deleted
         */
        bool removeStone(std::tuple<size_t, size_t> toRemove);
        /**
         * @return the size of the pool, that is the stones left to be drawn
         */
        size_t size();
    };
}

#endif //SRC_DOMINO_POOL_H

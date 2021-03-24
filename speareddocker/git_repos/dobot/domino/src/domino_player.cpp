//
// Created by lars on 26.08.19.
//
#include "../include/domino_player.h"

namespace domino {

    std::shared_ptr<DominoPlayer> resolvePlayer(Player p, std::shared_ptr<ros::NodeHandle> nh_ptr)
    {
        switch(p)
        {
            case AI_PLAYER:
                return std::make_shared<AIDominoPlayer>(AIDominoPlayer(nh_ptr));
            case HUMAN_PLAYER:
                return std::make_shared<HumanDominoPlayer>(HumanDominoPlayer());
        }
    }

	//--------------------------------------------------------------------------------------------------------------------------------
	//DominoPlayer

	DominoPlayer::DominoPlayer() {}

	size_t DominoPlayer::getNumberOfStones() const
	{
		return stones.size();
	}

	int DominoPlayer::count() const
	{
		if (getNumberOfStones() == 0)
			return WIN_POINTS;

		int points = 0;
		for (auto stone : stones)
			points += std::get<0>(stone) + std::get<1>(stone);

		return -points;
	}

	void DominoPlayer::initStones(const std::vector<std::tuple<size_t, size_t>> stones)
	{
		this->stones = stones;
	}

	void DominoPlayer::addStone(const std::tuple<size_t, size_t> stone)
	{
		stones.push_back(stone);
	}

	bool DominoPlayer::hasStone(const std::tuple<size_t, size_t> stone) const
	{
        return std::find(stones.begin(), stones.end(), stone) != stones.end();
    }

	void DominoPlayer::removeStone(std::tuple<size_t, size_t> stone)
	{
        stones.erase(std::remove(stones.begin(), stones.end(), stone), stones.end());
	}

	bool DominoPlayer::canLayStone(const domino::DominoField& field) const
	{
        for(auto stone : stones)
        {
            if(std::get<0>(stone) == field.getLeftEndValue() || std::get<0>(stone) == field.getRightEndValue()
               || std::get<1>(stone) == field.getLeftEndValue() || std::get<1>(stone) == field.getRightEndValue())
                return true;
        }

        return false;
	}

    //--------------------------------------------------------------------------------------------------------------------------------
	//AIDominoPlayer

	AIDominoPlayer::AIDominoPlayer(std::shared_ptr<ros::NodeHandle> nh_ptr) : DominoPlayer(),
	            arm(dobot::Dobot(dobot::dobot_names::DOBOT_RAIL)) {
        arm.initArm(nh_ptr, dobot::dobot_names::DOBOT_RAIL);
    }

	std::tuple<size_t, size_t> AIDominoPlayer::drawStone(domino::DominoStonePool stone_pool)
	{
        std::tuple<size_t, size_t> stone;
        //TODO: draw stone
        stone_pool.removeStone(stone);
	}

	std::tuple<std::tuple<size_t, size_t>, Position> AIDominoPlayer::selectStoneAndPosition(
            domino::DominoField field)
    {
        //TODO: implement
    }

    Orientation AIDominoPlayer::selectPlacement(domino::DominoField field, std::tuple<size_t, size_t> stone,
                                                domino::Position position)
    {
        //TODO: implement
    }

    void AIDominoPlayer::placeStone(std::tuple<size_t, size_t> stone, std::tuple<size_t, size_t> connectSide,
                                    std::tuple<size_t, size_t> endSide)
    {
        //TODO: implement
    }

	bool AIDominoPlayer::turn(DominoField& field, DominoStonePool& stone_pool)
	{
        int turn_try = 0;

        while(turn_try < MAX_TRIES && !canLayStone(field))
        {
            addStone(drawStone(stone_pool));
            turn_try++;
        }

        if (turn_try == MAX_TRIES && !canLayStone(field))
            return false;

        std::tuple<size_t, size_t> stone;
        Position position;
        Orientation orientation;

        do {
            std::tie(stone, position) = selectStoneAndPosition(field);
            orientation= selectPlacement(field, stone, position);
        }while(!hasStone(stone) && !field.placeStone(stone, position, orientation));

        std::tuple<size_t, size_t> connectSide, endSide;
        field.getLastInsertPosition(connectSide, endSide);
        placeStone(stone, connectSide, endSide);

        return true;
	}

	//--------------------------------------------------------------------------------------------------------------------------------
	//HumanDominoPlayer

	bool HumanDominoPlayer::turn(DominoField& field, DominoStonePool& stone_pool)
	{
		int turn_try = 0;
        size_t a, b;

		while (turn_try < MAX_TRIES && !canLayStone(field))
		{
			std::cout << "Please draw a stone and insert the stone drawn. Left side: ";
			std::cin >> a;
			std::cout << "Right side: ";
			std::cin >> b;
			auto drawn = a > b ? std::make_tuple(b,a) : std::make_tuple(a,b);
			addStone(drawn);
			stone_pool.removeStone(drawn);
			turn_try++;
		}

		if (turn_try == MAX_TRIES && !canLayStone(field))
			return false;

        std::tuple<size_t, size_t> stone;
        size_t position, orientation;
		do {
			std::cout << "Please insert the stone to lay. Left side: ";
			std::cin >> a;
			std::cout << "Right side: ";
			std::cin >> b;
			std::cout << "Please insert the position of the stone (left=0|right=1): ";
			std::cin >> position;
			std::cout << "Please insert the orientation of the stone (right=0|bottom=1|left=2|top=3):";
			std::cin >> orientation;


			//stones are stored using left <= right, thus swap if this does not hold.
			if (a > b)
				std::swap(a, b);

			stone = std::make_tuple(a,b);
		} while (!hasStone(stone) || !field.placeStone(stone, getPosition(position), getOrientation(orientation)));

		removeStone(stone);

		return true;
	}
}
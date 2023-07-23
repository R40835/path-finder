#include "bots.h"
#include "stdlib.h"

// redefining the function
void cAStarBot::ChooseNextGridPosition()
{
	// Manhattan distance:
	if (gAStar.max_length > 0)
	{
		// moving the bot according to the coordinates stored in our path_coordinates array
		SetNext(gAStar.path_coordinates[gAStar.max_length - 1][0], gAStar.path_coordinates[gAStar.max_length - 1][1], gLevel);
		gAStar.max_length -= 1;
	}
	// Euclidean distance:
	if (gAStar2.max_length > 0)
	{
		// moving the bot according to the coordinates stored in our path_coordinates array
		SetNext(gAStar2.path_coordinates[gAStar2.max_length - 1][0], gAStar2.path_coordinates[gAStar2.max_length - 1][1], gLevel);
		gAStar2.max_length -= 1;
	}
	// Diagonal distance:
	if (gAStar3.max_length > 0)
	{
		// moving the bot according to the coordinates stored in our path_coordinates array
		SetNext(gAStar3.path_coordinates[gAStar3.max_length - 1][0], gAStar3.path_coordinates[gAStar3.max_length - 1][1], gLevel);
		gAStar3.max_length -= 1;
	}
}

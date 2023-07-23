//======================================================================================
//Source code for a bot that randomly moves around, plus other bots as required
//
//(c) Patrick Dickinson, University of Lincoln, School of Computer Science, 2020
//======================================================================================

#include "bots.h"
#include "stdlib.h"

void cBotRandom::ChooseNextGridPosition() 
{
	//======================================================================================
	// We use gLevel to access environment layout
	// And gTarget to get target position
	// This function is called when a bot arrives at its current target location, to set the 
	// next grid position to reach the final target  
	//======================================================================================
	
	bool done = false;
	//Use while loop, as randomly selected location might not be valid
	while (!done)
	{
		int random = rand() % 4;
		if (random==0) done = SetNext((PositionX() + 1), (PositionY()), gLevel);
		else if (random == 1) done = SetNext((PositionX() - 1), (PositionY()), gLevel);
		else if (random == 2) done = SetNext((PositionX()), (PositionY() + 1), gLevel);
		else done = SetNext((PositionX()), (PositionY() - 1), gLevel);
	}
}
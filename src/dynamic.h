#pragma once
//======================================================================================
//Header for cDynamic class. This class is used both for the target/player, and also
//as a parent class for bots (cBotBase inherits from it).
//It captures the functionality for animating between one grid location and the next
//smoothly, and also rendering a sprite
//
//(c) Patrick Dickinson, University of Lincoln, School of Computer Science, 2020
//======================================================================================

#include "level.h"

class cDynamic
{
private:
	int currentX, currentY;
	int nextX, nextY;
	bool moving;
	int timerMs;
	bool initialised;
public:
	cDynamic();
	int moveTimeMs;
	//======================================================================================
	//Update function is called every frame: virtual so that child classes can be accessed 
	//through base class pointer
	//======================================================================================
	virtual void Update(int ms);
	//======================================================================================
	//Draw function called every frame update
	//======================================================================================
	void Draw(SDL_Texture* tex);
	bool isMoving() { return moving; }
	//======================================================================================
	//SetNext called to set next grid position to move to. SetCurrent is used for 
	//initialisation only  
	//======================================================================================
	bool SetNext(int x, int y, const cLevel& lvl); //return true if successful, false otherwise
	bool SetCurrent(int x, int y, const cLevel& lvl); //return true if successful
	//======================================================================================
	//Current Position
	//======================================================================================
	int PositionX() { return currentX; }
	int PositionY() { return currentY; }
};

//======================================================================================
//Global instance corresponds to a player controlled target object
//======================================================================================
extern cDynamic gTarget;

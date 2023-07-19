//======================================================================================
//Source code for dynamic object
//
//(c) Patrick Dickinson, University of Lincoln, School of Computer Science, 2020
//======================================================================================
#include "dynamic.h"
//======================================================================================
//Globals
//======================================================================================
cDynamic gTarget;

//======================================================================================
//constructor
//======================================================================================
cDynamic::cDynamic()
{
	currentX = currentY = nextX = nextY = 0;
	moving = false;
	timerMs = 0;
	moveTimeMs = 500; //time in miliseconds to move from one location to the next
	initialised = false;
}

//======================================================================================
//Update
//======================================================================================
void cDynamic::Update(int ms)
{
	if (!moving) return;
	timerMs += ms;
	if (timerMs >= moveTimeMs)
	{
		timerMs = 0;
		moving = false;
		currentX = nextX;
		currentY = nextY;
	}
}

//======================================================================================
//Draw function
//======================================================================================
void cDynamic::Draw(SDL_Texture* tex)
{
	float posX = currentX * 16;
	float posY = currentY * 16;
	if (moving)
	{
		float proportion = ((float)timerMs) / ((float)moveTimeMs);
		posX += ((nextX - currentX) * 16) * proportion;
		posY += ((nextY - currentY) * 16) * proportion;
	}

	SDL_Rect position;
	position.x = posX;
	position.y = posY;
	position.h = position.w = 16;
	SDL_RenderCopy(gMainRenderer, tex, NULL, &position);
}

//======================================================================================
// SetNext does some checks: must be not moving, valid target location, and adjacent
//to current location
//======================================================================================

bool cDynamic::SetNext(int x, int y, const cLevel& lvl)
{
	if (moving) return false;
	if (!lvl.isValid(x, y)) return false;
	if (x < (currentX - 1)) return false;
	if (x > (currentX + 1)) return false;
	if (y < (currentY - 1)) return false;
	if (y > (currentY + 1)) return false;

	nextX = x;
	nextY = y;
	moving = true;
	timerMs = 0.0f;
	return true;
}

bool cDynamic::SetCurrent(int x, int y, const cLevel& lvl)
{
	if (!initialised)
	{
		currentX = x;
		currentY = y;
		initialised = true;
		return true;
	}
	return false;
}




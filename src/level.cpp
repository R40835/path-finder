//======================================================================================
//Source code for cLevel class
//
//(c) Patrick Dickinson, University of Lincoln, School of Computer Science, 2020
//======================================================================================
#include "level.h"
#include "bots.h"
#include <stdio.h>

//======================================================================================
//Globals
//======================================================================================
cLevel gLevel;

//======================================================================================
//Valid if in range and position is traversable
//======================================================================================

bool cLevel::isValid(int x, int y) const
{
	if ((x < 0) || (x > 39)) return false;
	if ((y < 0) || (y > 39)) return false;
	int m = map[x][y];
	if (m != 0) return false;
	return true;
}

//======================================================================================
//Constructor
//======================================================================================
cLevel::cLevel()
{
	for (int i = 0; i < GRIDHEIGHT; i++)
	{
		for (int j = 0; j < GRIDWIDTH; j++)
		{
			map[i][j] = 0;
		}
	}
}

//======================================================================================
//Load map from ascii text file
//======================================================================================
void cLevel::Load(const char* fname)
{
	for (int i = 0; i < GRIDHEIGHT; i++)
	{
		for (int j = 0; j < GRIDWIDTH; j++)
		{
			map[i][j] = 0;
		}
	}
	FILE* file = fopen(fname, "rt");
	for (int j = 0; j < GRIDHEIGHT; j++)
	{
		char line[128];
		fread(line, 1, 41, file);
		for (int i = 0; i < GRIDWIDTH; i++)
		{
			if (line[i] == '#') map[i][j] = 1;
			else map[i][j] = 0;
		}
	}
	fclose(file);
}

//======================================================================================
//Draw grid
//======================================================================================

void cLevel::Draw(int select_function) // the function takes one argument now to enable the user to choose between functions
{
	SDL_Rect position;
	position.h = position.w = 16;

	for (int i = 0; i < GRIDHEIGHT; i++)
	{
		for (int j = 0; j < GRIDWIDTH; j++)
		{
			position.x = i * 16;
			position.y = j * 16;

			if (select_function == 1) // Manhattan distance will be drawn
			{
				if (map[i][j] == 0)
				{
					if ((gAStar.completed) && (gAStar.inPath[i][j] == true))
						SDL_RenderCopy(gMainRenderer, tileRouteTexture, NULL, &position);
					else if ((gAStar.completed) && (gAStar.closed[i][j] == true))
						SDL_RenderCopy(gMainRenderer, tileClosedTexture, NULL, &position);
					else
						SDL_RenderCopy(gMainRenderer, tileTexture, NULL, &position);
				}
				else
					SDL_RenderCopy(gMainRenderer, tileBlockedTexture, NULL, &position);
			}
			else if (select_function == 2) // Euclidean distance will be drawn
			{
				if (map[i][j] == 0)
				{
					if ((gAStar2.completed) && (gAStar2.inPath[i][j] == true))
						SDL_RenderCopy(gMainRenderer, tileRouteTexture, NULL, &position);
					else if ((gAStar2.completed) && (gAStar2.closed[i][j] == true))
						SDL_RenderCopy(gMainRenderer, tileClosedTexture, NULL, &position);
					else SDL_RenderCopy(gMainRenderer, tileTexture, NULL, &position);
				}
				else SDL_RenderCopy(gMainRenderer, tileBlockedTexture, NULL, &position);
			}
			else if (select_function == 3) // diagonal distance will be drawn
			{
				if (map[i][j] == 0)
				{
					if ((gAStar3.completed) && (gAStar3.inPath[i][j] == true))
						SDL_RenderCopy(gMainRenderer, tileRouteTexture, NULL, &position);
					else if ((gAStar3.completed) && (gAStar3.closed[i][j] == true))
						SDL_RenderCopy(gMainRenderer, tileClosedTexture, NULL, &position);
					else SDL_RenderCopy(gMainRenderer, tileTexture, NULL, &position);
				}
				else SDL_RenderCopy(gMainRenderer, tileBlockedTexture, NULL, &position);
			}

			else
			{
				if (map[i][j] == 0)
				{
					SDL_RenderCopy(gMainRenderer, tileTexture, NULL, &position);
				}
				else
					SDL_RenderCopy(gMainRenderer, tileBlockedTexture, NULL, &position);
			}
		}
	}
}

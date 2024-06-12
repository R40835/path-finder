#include "bots.hpp"
#include "stdlib.h"
#include "iostream"

using namespace std;

cDijkstra::cDijkstra() { 
	for (int i = 0; i < GRIDWIDTH; i++) 
	{
		for (int j = 0; j < GRIDHEIGHT; j++) 
		{
			closed[i][j] = false;
			cost[i][j] = 1000000.0f;
			linkX[i][j] = -1;
			linkY[i][j] = -1;
			inPath[i][j] = false;
		}
	}
	completed = false;
}

void cDijkstra::UpdateNeighbors(int min_x, int min_y, float base_cost) { 
	float straight_cost{ 1.0 };
	float diagonal_cost{ 1.4 }; 
	// Straight moves
	if (gLevel.isValid(min_x - 1, min_y) && !closed[min_x - 1][min_y] &&
		(base_cost + straight_cost) < cost[min_x - 1][min_y])
	{
		cost[min_x - 1][min_y] = cost[min_x][min_y] + 1.0;
		linkX[min_x - 1][min_y] = min_x;
		linkY[min_x - 1][min_y] = min_y;
	}
	if (gLevel.isValid(min_x, min_y - 1) && !closed[min_x][min_y - 1] &&
		(base_cost + straight_cost) < cost[min_x][min_y - 1])
	{
		cost[min_x][min_y - 1] = cost[min_x][min_y] + 1.0;
		linkX[min_x][min_y - 1] = min_x;
		linkY[min_x][min_y - 1] = min_y;
	}
	if (gLevel.isValid(min_x, min_y + 1) && !closed[min_x][min_y + 1] &&
		(base_cost + straight_cost) < cost[min_x][min_y + 1])
	{
		cost[min_x][min_y + 1] = cost[min_x][min_y] + 1.0;
		linkX[min_x][min_y + 1] = min_x;
		linkY[min_x][min_y + 1] = min_y; 
	}
	if (gLevel.isValid(min_x + 1, min_y) && !closed[min_x + 1][min_y] &&
		(base_cost + straight_cost) < cost[min_x + 1][min_y])
	{
		cost[min_x + 1][min_y] = cost[min_x][min_y] + 1.0;
		linkX[min_x + 1][min_y] = min_x;
		linkY[min_x + 1][min_y] = min_y;
	}
	// Diagonal moves
	if (gLevel.isValid(min_x + 1, min_y - 1) && !closed[min_x + 1][min_y - 1] &&
		(base_cost + diagonal_cost) < cost[min_x + 1][min_y - 1])
	{
		cost[min_x + 1][min_y - 1] = cost[min_x][min_y] + 1.4;
		linkX[min_x + 1][min_y - 1] = min_x;
		linkY[min_x + 1][min_y - 1] = min_y;
	}
	if (gLevel.isValid(min_x + 1, min_y + 1) && !closed[min_x + 1][min_y + 1] &&
		(base_cost + diagonal_cost) < cost[min_x + 1][min_y + 1])
	{
		cost[min_x + 1][min_y + 1] = cost[min_x][min_y] + 1.4;
		linkX[min_x + 1][min_y + 1] = min_x;
		linkY[min_x + 1][min_y + 1] = min_y;
	}
	if (gLevel.isValid(min_x - 1, min_y + 1) && !closed[min_x - 1][min_y + 1] &&
		(base_cost + diagonal_cost) < cost[min_x - 1][min_y + 1])
	{
		cost[min_x - 1][min_y + 1] = cost[min_x][min_y] + 1.4;
		linkX[min_x - 1][min_y + 1] = min_x;
		linkY[min_x - 1][min_y + 1] = min_y;
	}
	if (gLevel.isValid(min_x - 1, min_y - 1) && !closed[min_x - 1][min_y - 1] &&
		(base_cost + diagonal_cost) < cost[min_x - 1][min_y - 1])
	{
		cost[min_x - 1][min_y - 1] = cost[min_x][min_y] + 1.4;
		linkX[min_x - 1][min_y - 1] = min_x;
		linkY[min_x - 1][min_y - 1] = min_y;
	}
}

int cDijkstra::TracePath(cBotBase& bot) { 
	bool done = false; // Set to true when we are back at the bot position
	int nextClosedX = gTarget.PositionX(); // Start of path
	int nextClosedY = gTarget.PositionY(); // Start of path
	int path = 0; // To move in our path

	while (!done) {
		inPath[nextClosedX][nextClosedY] = true;
		path_coordinates[path][0] = nextClosedX;
		path_coordinates[path][1] = nextClosedY;
		int tmpX = nextClosedX;
		int tmpY = nextClosedY;
		path += 1;
		nextClosedX = linkX[tmpX][tmpY];
		nextClosedY = linkY[tmpX][tmpY];
		if ((nextClosedX == bot.PositionX()) && (nextClosedY == bot.PositionY())) {
			done = true;
		}
	}
	return path;
}

void cDijkstra::Build(cBotBase& bot)
{
	// the bot starts the search
	cost[bot.PositionX()][bot.PositionY()] = 0;
	// finding the lowest cost
	while (!closed[gTarget.PositionX()][gTarget.PositionY()])
	{
		float min_cost = 1000000.0f;
		int min_x = 0;
		int min_y = 0;

		for (int i = 0; i < GRIDWIDTH; i++)
		{
			for (int j = 0; j < GRIDHEIGHT; j++)
			{
				if ((cost[i][j] <= min_cost) && gLevel.isValid(i, j) && !closed[i][j])
				{
					min_cost = cost[i][j];
					min_x = i;
					min_y = j;
				}
			}
		}
		closed[min_x][min_y] = true;
		UpdateNeighbors(min_x, min_y, cost[min_x][min_y]);
	}
}

// defining the build function for A*, this one has the Manhattan distance implemented in, we'll redifine other functions to use the other
// functions as well.
void cAStar::Build(cBotBase& bot)
{
	cost[bot.PositionX()][bot.PositionY()] = 0;

	while (!closed[gTarget.PositionX()][gTarget.PositionY()])
	{
		int min_x = 0;
		int min_y = 0;
		float min_cost = 1000000.0f;
		for (int i = 0; i < GRIDWIDTH; i++)
		{
			for (int j = 0; j < GRIDHEIGHT; j++)
			{
				//Manhatan distance
				float heuristic = fabs(gTarget.PositionX() - i) + fabs(gTarget.PositionY() - j);

				if (((cost[i][j] + heuristic) <= min_cost) && gLevel.isValid(i, j) && !closed[i][j])
				{
					min_cost = cost[i][j] + heuristic;
					min_x = i;
					min_y = j;
				}
			}
		}
		closed[min_x][min_y] = true;
		UpdateNeighbors(min_x, min_y, cost[min_x][min_y]);
	}	
	max_length = TracePath(bot); 
	completed = true; 
}

// for the use of euclidean distance function we redefine the build function
void cAStar2::Build(cBotBase& bot)
{
	cost[bot.PositionX()][bot.PositionY()] = 0;

	while (!closed[gTarget.PositionX()][gTarget.PositionY()])
	{
		int min_x = 0;
		int min_y = 0;
		float min_cost = 1000000.0f;
		for (int i = 0; i < GRIDWIDTH; i++)
		{
			for (int j = 0; j < GRIDHEIGHT; j++)
			{
				//Euclidean distance
				float heuristic = sqrt(pow(fabs(gTarget.PositionX() - i), 2) + pow(fabs(gTarget.PositionY() - j), 2)); 
				if (((cost[i][j] + heuristic) <= min_cost) && gLevel.isValid(i, j) && !closed[i][j])
				{
					min_cost = cost[i][j] + heuristic;
					min_x = i;
					min_y = j;
				}
			}
		}
		closed[min_x][min_y] = true;
		UpdateNeighbors(min_x, min_y, cost[min_x][min_y]);
	}
	max_length = TracePath(bot); 
	completed = true; 
}

void cAStar3::Build(cBotBase& bot)
{
	cost[bot.PositionX()][bot.PositionY()] = 0;

	while (!closed[gTarget.PositionX()][gTarget.PositionY()])
	{
		int min_x = 0;
		int min_y = 0;
		float min_cost = 1000000.0f;
		for (int i = 0; i < GRIDWIDTH; i++)
		{
			for (int j = 0; j < GRIDHEIGHT; j++)
			{
				//Diagonal distance
				float heuristic = (fabs(gTarget.PositionX() - i) + (fabs(gTarget.PositionY() - j)) -
					0.6 * std::min(fabs(gTarget.PositionX() - i), fabs(gTarget.PositionY() - j)));     
				if (((cost[i][j] + heuristic) <= min_cost) && gLevel.isValid(i, j) && !closed[i][j])
				{
					min_cost = cost[i][j] + heuristic;
					min_x = i;
					min_y = j;
				}
			}
		}
		closed[min_x][min_y] = true;
		UpdateNeighbors(min_x, min_y, cost[min_x][min_y]);
	}
	max_length = TracePath(bot); 
	completed = true; 
}

// instantiating global objects
cDijkstra gDijkstra; 
cAStar gAStar;
cAStar2 gAStar2;
cAStar3 gAStar3;
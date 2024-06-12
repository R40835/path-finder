#pragma once
//======================================================================================
//Header file for bots: initially cBotRandom is defined here, but other bot classes
//can go here too
//
//(c) Patrick Dickinson, University of Lincoln, School of Computer Science, 2020
//======================================================================================

#include "botbase.hpp" 


class cBotRandom : public cBotBase
{
	virtual void ChooseNextGridPosition();
};


class cDijkstra
{
public:
	bool closed[GRIDWIDTH][GRIDHEIGHT];
	float cost[GRIDWIDTH][GRIDHEIGHT];
	int linkX[GRIDWIDTH][GRIDHEIGHT];
	int linkY[GRIDWIDTH][GRIDHEIGHT];
	bool inPath[GRIDWIDTH][GRIDHEIGHT];
	int path_coordinates[100][2];
	bool completed;

	// methods
	virtual void Build(cBotBase& bot);
	int TracePath(cBotBase& bot);  
	void UpdateNeighbors(int min_x, int min_y, float base_cost); 

	// constructor
	cDijkstra();
};

// this class use manhattan by clicking on P
class cAStar : public cDijkstra
{
public:
	virtual void Build(cBotBase& bot);
	// initialising path length as 0
	int max_length = 0;
};

// creating our bot from this class that inherit from cBot base so that it can move
class cAStarBot : public cBotBase
{
	virtual void ChooseNextGridPosition();
};

// this class uses Euclidean distance by clicking on A 
class cAStar2 : public cDijkstra
{
public:
	virtual void Build(cBotBase& bot);
	// initialising path length as 0
	int max_length = 0;
};

// this class uses diagonal distance by clicking on B

class cAStar3 : public cDijkstra
{
public:
	virtual void Build(cBotBase& bot);
	// initialising path length as 0
	int max_length = 0;
};

// extern references:
extern cDijkstra gDijkstra;
extern cAStar gAStar;
extern cAStar2 gAStar2;
extern cAStar3 gAStar3;


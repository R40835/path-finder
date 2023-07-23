#pragma once
//======================================================================================
//Header file for bots: initially cBotRandom is defined here, but other bot classes
//can go here too
//
//(c) Patrick Dickinson, University of Lincoln, School of Computer Science, 2020
//======================================================================================

#include "botbase.h"


class cBotRandom : public cBotBase
{
	virtual void ChooseNextGridPosition();
};

// creating Dijkstra class

class cDijkstra
{
public:
	// declaring arrays that we'll use later on

	bool closed[GRIDWIDTH][GRIDHEIGHT];
	float cost[GRIDWIDTH][GRIDHEIGHT];
	int linkX[GRIDWIDTH][GRIDHEIGHT];
	int linkY[GRIDWIDTH][GRIDHEIGHT];
	bool inPath[GRIDWIDTH][GRIDHEIGHT];
	bool completed;

	// method
	virtual void Build(cBotBase& bot);

	// constructor
	cDijkstra() { completed = false; }
};

// this class use manhattan by clicking on P
class cAStar : public cDijkstra
{
public:
	virtual void Build(cBotBase& bot);
	// our array to store the path
	int path_coordinates[100][2];
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
	// our array to store the path
	int path_coordinates[100][2];
	// initialising path length as 0
	int max_length = 0;
};

// this class uses diagonal distance by clicking on B

class cAStar3 : public cDijkstra
{
public:
	virtual void Build(cBotBase& bot);
	// our array to store the path
	int path_coordinates[100][2];
	// initialising path length as 0
	int max_length = 0;
};

// extern references:

extern cDijkstra gDijkstra;
extern cAStar gAStar;
extern cAStar2 gAStar2;
extern cAStar3 gAStar3;


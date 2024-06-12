#pragma once
//======================================================================================
//Header for game level = grid environment
//
//(c) Patrick Dickinson, University of Lincoln, School of Computer Science, 2020
//======================================================================================
#include "game.hpp" 

//======================================================================================
//Level class
//======================================================================================
class cLevel
{
	//======================================================================================
	//Grid is just a 2D array. Note that array is indexed [row][Column]
	//======================================================================================
	int map[GRIDWIDTH][GRIDHEIGHT];
public:
	cLevel();
	//======================================================================================
	//Loads an ascii text file describing layout
	//======================================================================================
	void Load(const char* fname);
	//======================================================================================
	//Draw entire grid in 640x640 window
	//======================================================================================
	
	// in order to opt for a function in main.cpp (Manhattan, Euclidean, Diagonal), we proceed to add an argument to the draw function 
	void Draw(int select_function);

	//======================================================================================
	//Return whether a specified grid location is blocked/valid
	//valid = not blocked and in range 0 to 39
	//======================================================================================
	bool isBlocked(int x, int y) const { return(map[x][y] != 0); }
	bool isValid(int x, int y) const;
};

//======================================================================================
//Global level object 
//======================================================================================
extern cLevel gLevel;

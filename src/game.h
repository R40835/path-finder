#pragma once
//======================================================================================
//Header for global game objects. Sorry, yes, there are globals.
//
//(c) Patrick Dickinson, University of Lincoln, School of Computer Science, 2020
//======================================================================================

#include "SDL.h"
//======================================================================================
//Global quit flag to exit the game
//======================================================================================
extern bool gQuit;
//======================================================================================
//SDL Main constructs
//======================================================================================
extern SDL_Window* gMainWindow;
extern SDL_Renderer* gMainRenderer;
//======================================================================================
//Surfaces and textures for sprites
//======================================================================================
extern SDL_Surface* tileSurface;
extern SDL_Surface* tileBlockedSurface;
extern SDL_Surface* targetSurface;
extern SDL_Surface* botSurface;
extern SDL_Surface* tileClosedSurface;
extern SDL_Surface* tileRouteSurface;

extern SDL_Texture* tileTexture;
extern SDL_Texture* tileBlockedTexture;
extern SDL_Texture* targetTexture;
extern SDL_Texture* botTexture;
extern SDL_Texture* tileClosedTexture;
extern SDL_Texture* tileRouteTexture;

//======================================================================================
//Macros for grid size 
//======================================================================================
#define GRIDWIDTH (40)
#define GRIDHEIGHT (40)




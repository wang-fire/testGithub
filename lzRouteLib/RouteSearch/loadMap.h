#ifndef LOADMAP_H
#define LOADMAP_H

#include"./common.h" 
#include<string>
#include<string.h>

int cLoadMap( const char* fileName, const char* turnInfoFileName, const char* offsetInfoFileName, const char* speedInfoFileName );
int cCloseMap();

#endif

//openList, closeList
#ifndef ASTARSEARCH_H
#define ASTARSEARCH_H

#include"./common.h"
#include<vector>
#include<queue>

int getKeyNodePath( const TaskInfoSearch& taskContent, std::vector<int>& keyNodePathVec );
int showKeyNodePath(const std::vector<int> &keyNodePath);

#endif

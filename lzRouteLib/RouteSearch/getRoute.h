#ifndef GET_ROUTE_H
#define GET_ROUTE_H

#include"./common.h"

int32 getRoute( const TaskInfo& taskContentRecv, const char* scriptFileName, std::vector<int32>& agvForkKeyNodePathVec, std::vector<PathNode>& agvForkPathVec);

#endif

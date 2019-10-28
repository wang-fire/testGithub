#ifndef AGVFORKPATH2STANDARDPATH_H 
#define AGVFORKPATH2STANDARDPATH_H

#include"./common.h"
#include"./agvForkParam.h"
#include<vector>

int32 convertAgvForkPath2StandardPath(const TaskInfo& taskContentRecv, std::vector<PathNode>& agvForkPathVec);


#endif

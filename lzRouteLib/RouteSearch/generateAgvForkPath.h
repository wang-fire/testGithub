#ifndef GENERATE_AGV_FORK_PATH_H
#define GENERATE_AGV_FORK_PATH_H

#include"./agvForkParam.h"
#include"./common.h"

int32 generateAgvForkPath(const TaskInfoGenerate& taskContentGenerate, const std::vector<int32>& keyNodePathVec, std::vector<PathNode>& agvForkPathVec);

#endif

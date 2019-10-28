#ifndef LZROUTELIB_H
#define LZROUTELIB_H

#include "lzroutelib_global.h"
#include "./RouteSearch/common.h"

class LZROUTELIBSHARED_EXPORT LzRouteLib
{

public:
    LzRouteLib();
    int lGetRoute(const TaskInfo& taskInfo, const char* scriptFileName, std::vector<int32>& agvKeyPath, std::vector<PathNode>& agvPath);
    int lLoadMap(const char* fileName, const char* turnInfoFileName, const char* offsetInfoFileName, const char* speedInfoFileName);
};

#endif // LZROUTELIB_H

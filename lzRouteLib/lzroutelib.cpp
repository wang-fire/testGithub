#include "lzroutelib.h"
#include "./RouteSearch/getRoute.h"
#include "./RouteSearch/loadMap.h"

LzRouteLib::LzRouteLib()
{
}

int LzRouteLib::lGetRoute(const TaskInfo& taskInfo, const char* scriptFileName, std::vector<int32>& agvKeyPath, std::vector<PathNode>& agvPath)
{
    int ret = getRoute(taskInfo, scriptFileName, agvKeyPath, agvPath );
    return ret;
}

int LzRouteLib::lLoadMap(const char *fileName, const char *turnInfoFileName, const char *offsetInfoFileName, const char *speedInfoFileName)
{
    int ret = cLoadMap( fileName, turnInfoFileName, offsetInfoFileName, speedInfoFileName);
    return ret;
}

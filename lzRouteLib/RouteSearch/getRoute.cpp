#include"./getRoute.h"
#include"./AStarSearch.h"
#include"./generateAgvForkPath.h"
#include"./agvForkPath2StandardPath.h"
#include"./moveScript.h"

int32 getRoute( const TaskInfo& taskInfo, const char* scriptFileName, std::vector<int32>& agvKeyPath, std::vector<PathNode>& agvPath){

   agvKeyPath.clear();
   agvPath.clear();

   TaskInfoSearch taskContentSearch;
   taskContentSearch.agvHeadTheta = taskInfo.agvHeadTheta;
   taskContentSearch.id_1 = taskInfo.id_1;
   taskContentSearch.id_2 = taskInfo.id_2;

   int ret = getKeyNodePath(taskContentSearch, agvKeyPath ); //生成关键节点路径，路径中的每个点的id存在agvForkKeyNodePathVec， id作为keyNodes数组的
   if (ret < 0){                                                  //索引，可以在keyNodes数组里面索引路径节点的坐标等信息
     printf("key path generate failed!\n");
     return ret;
   }

   TaskInfoGenerate taskContentGenerate;
   taskContentGenerate.turnRadius = taskInfo.turnRadius;
   taskContentGenerate.agvHeadTheta = taskInfo.agvHeadTheta;

   taskContentGenerate.routeOffsetX = taskInfo.taskOffsetInfo.routeOffsetX;
   taskContentGenerate.routeOffsetY = taskInfo.taskOffsetInfo.routeOffsetY;

   ret = generateAgvForkPath(taskContentGenerate, agvKeyPath, agvPath);
   if (ret < 0){
    //printf("path generate failed!\n");
    return ret;
   }

   convertAgvForkPath2StandardPath(taskInfo, agvPath);

   //直接对改脚本进行处理
   ret = movScriptProcess( agvPath, scriptFileName );
   if( ret < 0 ){
      //qDebug()<<"script file process failed!";
      return ret;
   }

   for( unsigned int i = 0; i < agvPath.size(); i++ ){
      PathNode& node = agvPath[i];
      node.agvId = taskInfo.agvId;
   }

   return SUCCEED;
}

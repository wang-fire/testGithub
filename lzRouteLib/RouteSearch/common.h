#ifndef COMMON_H
#define COMMON_H

#include<stdlib.h>
#include<stdio.h>
#include<math.h>
#include<vector>
#include"./typeDefines.h"
#include"./agvForkParam.h"
#include"./RetType.h"

const uint32 MAXIN = 4;
const uint32 MAXOUT = 4;
const int INF = 1 << 30;
const float PI = (float)3.14159;
const uint32 gMaxTurnNum = 300;
const uint32 gMaxOffsetNum = 300;

const float defaultLINEspeed = 0.8;
const float defaultCURVEspeed = 0.6;

const float lastLineSpeed = 0.4;
const float lastCurveSpeed = 0.4;

const int TURNDIR = 0;
const int rSPrintDebugInfo = 0;

typedef struct Point_{
    float x;
    float y;
    Point_():x(0.0),y(0.0){}
}Point;

typedef struct RouteMapElement_{
    int id;
    float x;
    float y;
    int id_1;
    int id_2;
    int id_3;
    int id_4;
    RouteMapElement_():id(0),x(0.0), y(0.0), id_1(0), id_2(0), id_3(0), id_4(0){}
}RouteMapElement;

typedef struct RouteMap_{
    int maxNodeIndex;
    int nodesNum;
    int edgesNum;
    RouteMapElement* rtMapElements;
    RouteMap_():maxNodeIndex(0), nodesNum(0), edgesNum(0), rtMapElements(NULL){}
}RouteMap;

typedef struct KeyNode_{
    uint32 id;
    uint32 parrentId;
    uint32 inNode[MAXIN];
    uint32 outNode[MAXOUT];
    float32 fScore;
    float32 gScore;
    float32 hScore;
    Point pt;
    bool isInCloseList;
    bool isInOpenList;
    bool isChosen; //����ͼ�α༭ʱ���жϸõ��Ƿ���ѡ��
    char type;
    char layer;
    KeyNode_(){
        id = 0;
        parrentId = 0;
        inNode[0] = 0;
        inNode[1] = 0;
        inNode[2] = 0;
        inNode[3] = 0;
        outNode[0] = 0;
        outNode[1] = 0;
        outNode[2] = 0;
        outNode[3] = 0;
        fScore = 0.0;
        hScore = 0.0;
        gScore = 0.0;
        isInOpenList = 0;
        isInCloseList = 0;
        isChosen = 0;
        type = 1;
        layer = 1;
    }
} KeyNode;

typedef struct PathNode_{
    uint32 agvId;
    uint32 id;
    int32 moveDirection;
    int32 turnDirection;
    int32 startPointVelocityMode;
    int32 endPointVelocityMode;
    int32 forkControlType;
    int32 action;
    uint32 layer;
    uint32 nodeOrder;
    float32 height; //�߶��ú��ױ�ʾ
    float32 dTheta;
    float32 agvHeadTheta;
    float32 speed;
    float32 ptCenterX;
    float32 ptCenterY;
    Point pt;
    PathNode_():agvId(0), id(0), startPointVelocityMode(AGV_FORK_START_POINT_VELOCITY_ACCELERATE), \
                endPointVelocityMode(2), forkControlType(2), action(AGV_FORK_ACTION_NONE_LOCK), \
                 layer(0), height(0), dTheta(0.0), agvHeadTheta(0.0), speed(1.5), ptCenterX(0.0), ptCenterY(0.0) {
    }
} PathNode;

typedef struct EDGE_{
    int id;
    int turnDir;
    EDGE_():id(0), turnDir(TURNDIR) {
    }

} EDGE;

typedef struct PositionOffset {
    float dx;
    float dy;
}PositionOffset;

typedef struct TurnInfo_{
    int id_1;
    int id_2;
    int id_3;
    int turnType;
    float radius;
    TurnInfo_():id_1(0), id_2(0), id_3(0), turnType(0), radius(1.5){}
} TurnInfo;

typedef struct OffsetInfo_{
    int agvId;
    int benchId;
    int offsetX;
    int offsetY;
}OffsetInfo;

typedef struct SpeedInfo_{
    int id_1;
    int id_2;
    int id_3;
    float speed;
    SpeedInfo_():id_1(0), id_2(0), id_3(0){}
}SpeedInfo;

typedef struct TaskOffsetInfo_{
     bool bOffset;
    float posOffsetX;
    float posOffsetY;
    float routeOffsetX;
    float routeOffsetY;
    TaskOffsetInfo_():bOffset(false), posOffsetX(0.0), posOffsetY(0.0), routeOffsetX(0.0), routeOffsetY(0.0){}
}TaskOffsetInfo;

typedef struct ForkWorkType_{
    int workType;
    int heightLiftFork;
    int heightLiftGoods;
    int heightWork;
    float distance;
    ForkWorkType_():workType(0), heightLiftFork(0), heightLiftGoods(0), heightWork(0), distance(0.0){}
}ForkWorkType;

#pragma pack(push)
#pragma pack(4)
typedef struct TaskInfo{
    bool bHaveNewTask;
    int agvId;
    int taskId;
    int id_1;
    int id_middle;
    int id_2;
    float agvHeadTheta;

    int agvForkOutType;
    float turnRadius;

    int actIndex;

    ForkWorkType forkWorkType;
    TaskOffsetInfo taskOffsetInfo; //contain route offset info and position offset info

}TaskInfo;
#pragma pack(pop)

typedef struct TaskInfoSearch_{
    int id_1;
    int id_2;
    float agvHeadTheta;
}TaskInfoSearch;

typedef struct TaskInfoGenerate_{
    int agvForkOutType; //if outType = 1, the agv Fork will start move with its head, if outType = -1, agv Fork will start move with its tail
    float turnRadius; //the turn radius of the fork
    float agvHeadTheta;
    float routeOffsetX;
    float routeOffsetY;
}TaskInfoGenerate;

typedef struct _routeKeyNode{
    int index;
    float x;
    float y;
}RouteKeyNode;

extern EDGE** edges;
extern KeyNode* keyNodes;
extern TurnInfo turnInfo[];
extern SpeedInfo speedInfo[];
extern OffsetInfo offsetInfo[];
extern uint32 nodesNum;
extern uint32 edgesNum;

#endif

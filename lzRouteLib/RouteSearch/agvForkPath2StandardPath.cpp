
#include"./agvForkPath2StandardPath.h"
#include"./geometryCalculate.h"
#include<QDebug>


static int32 setMoveMode(std::vector<PathNode>& agvForkPathVec, int forkMoveOutType ){
    std::vector<PathNode>::iterator it = agvForkPathVec.begin();
    int sign = 1;
    if(forkMoveOutType == AGV_FORK_MOVE_OUT_TAIL ){
        sign = -1;
    }
    //调整方向
    for(; it != agvForkPathVec.end(); it++){
        it->moveDirection = (it->moveDirection) * sign;
    }

    //映射方向
    it = agvForkPathVec.begin();
    for (; it != agvForkPathVec.end(); it++){
        if (it->moveDirection == 1){
            it->moveDirection = AGV_FORK_MOVE_FORWARD;
        }
        else if (it->moveDirection == -1){
            it->moveDirection = AGV_FORK_MOVE_BACKWARD;
        }
        else if (it->moveDirection == 0){
            it->moveDirection = AGV_FORK_MOVE_STOPMOVE;
        }
        if (it->turnDirection == 1){
            it->turnDirection = AGV_FORK_TURN_DIRECTION_LEFT;
        }
        else if (it->turnDirection == -1){
            it->turnDirection = AGV_FORK_TURN_DIRECTION_RIGHT;
        }
        else if ( it->turnDirection == 0){
            it->turnDirection = AGV_FORK_TURN_NONE;
        }
    }
    return 1;
}


static int32 setLayerMode(std::vector<PathNode>& agvForkPathVec){
	for (auto pNode : agvForkPathVec){
		pNode.layer = keyNodes[pNode.id].layer;
	}
	return 1;
}

/*
static int32 setHeightMode(std::vector<PathNode>& agvForkPathVec, int32 height){
	for (auto pNode : agvForkPathVec){
		pNode.height = height;
	}
}
*/

/*
static int32 setNodeSpeedMode(std::vector<PathNode>& agvForkPathVec){
    for (auto& pNode : agvForkPathVec){
		pNode.startPointVelocityMode = AGV_FORK_START_POINT_VELOCITY_ACCELERATE;
		pNode.endPointVelocityMode = AGV_FORK_END_POINT_VELOCITY_DECELERATE;
	}
    for (auto& pNode : agvForkPathVec){
		if (pNode.turnDirection != AGV_FORK_TURN_NONE){
			pNode.endPointVelocityMode = AGV_FORK_END_POINT_VELOCITY_CONSTSPEED;
		}
	}
	return 1;
}
*/

/*
static int32 setAgvForkHeadTheta(taskInfoGenerate taskContentGenerate, std::vector<PathNode>& agvForkPathVec){
	auto it_start = agvForkPathVec.begin();
	auto it_end = agvForkPathVec.end();
	it_start->agvHeadTheta = taskContentGenerate.agvHeadTheta;
	++it_start;
	for (auto it = it_start; it < it_end; it++){
		it->agvHeadTheta = (it - 1)->agvHeadTheta + it->dTheta;
	}
	return 1;
}
*/

/*
static int32 setAgvForkHeadTheta2(std::vector<PathNode>& agvForkPathVec)
{
	//������������ͬ�ĵ���Ϊ���㣬��m_taskPath�е�ǰ����Ԫ�ص����겻ͬ
	if (agvForkPathVec.size() < 2){
		return 0;
	}
	Point& pt_1 = agvForkPathVec[0].pt;
	Point& pt_2 = agvForkPathVec[1].pt;
	float start_theta = (int)(100 * atan2(pt_2.y - pt_1.y, pt_2.x - pt_1.x));
	start_theta = start_theta / 100;

	agvForkPathVec[0].agvHeadTheta = start_theta;
    for (unsigned int i = 1; i < agvForkPathVec.size(); i++){
		int temp = 100 * agvForkPathVec[i].dTheta;
		agvForkPathVec[i].dTheta = ((float)temp) / 100;
        if( agvForkPathVec[i].dTheta > PI ){
            agvForkPathVec[i].dTheta -= 2*PI;
        }
        else if(agvForkPathVec[i].dTheta < -PI){
            agvForkPathVec[i].dTheta += 2*PI;
        }

		if (agvForkPathVec[i].dTheta > 1.57) //the turn theta must be less than 90�� or larger than -90��, and the rule must be obeyed
			agvForkPathVec[i].dTheta = 1.569;
		else if (agvForkPathVec[i].dTheta < -1.569)
			agvForkPathVec[i].dTheta = -1.569;

		agvForkPathVec[i].agvHeadTheta = agvForkPathVec[i - 1].agvHeadTheta + agvForkPathVec[i].dTheta;
		if (agvForkPathVec[i].agvHeadTheta > 3.14){
			agvForkPathVec[i].agvHeadTheta -= (float32)2 * 3.14;
		}
		else if (agvForkPathVec[i].agvHeadTheta < -3.14){
			agvForkPathVec[i].agvHeadTheta += (float32)2 * 3.14;
		}

	}
	return 1;
}
*/

/*
    切点角度计算问题
*/

/*
    车肯定沿直线方向走动
*/

static int32 setAgvForkHeadTheta(std::vector<PathNode>& agvForkPathVec)
{
    if (agvForkPathVec.size() < 2){
        return 0;
    }
   // float startTheta = atan2(agvForkPathVec[1].pt.y - agvForkPathVec[0].pt.y, agvForkPathVec[1].pt.x - agvForkPathVec[0].pt.x);
   // agvForkPathVec[0].agvHeadTheta = startTheta;


    float bias = 0.02; //20mm的一个阈值，判断是不是同一个点
    for( unsigned int i = 1; i < agvForkPathVec.size(); i++){

        int turnDirection = agvForkPathVec[i].turnDirection;
        Point ptStart;
        Point ptEnd;
        float lineTheta;
       // PathNode pNode;
        float dist = fabs(agvForkPathVec[i-1].pt.x - agvForkPathVec[i].pt.x) + fabs(agvForkPathVec[i-1].pt.y - agvForkPathVec[i].pt.y);
        if(dist < bias ){
            lineTheta = agvForkPathVec[i-1].agvHeadTheta;
        }
        else{
            if(turnDirection == AGV_FORK_TURN_DIRECTION_LEFT || turnDirection == AGV_FORK_TURN_DIRECTION_RIGHT ){
                if(i + 1 < agvForkPathVec.size() ){
                    ptStart = agvForkPathVec[i].pt;
                    ptEnd = agvForkPathVec[i + 1].pt;
                    lineTheta = atan2(ptEnd.y - ptStart.y, ptEnd.x - ptStart.x);
                }
                else{
                    lineTheta = agvForkPathVec[i-1].agvHeadTheta + agvForkPathVec[i].dTheta;
                }
            }
            else{
                ptStart = agvForkPathVec[i-1].pt;
                ptEnd = agvForkPathVec[i].pt;
                lineTheta = atan2(ptEnd.y - ptStart.y, ptEnd.x - ptStart.x);
            }

            if(agvForkPathVec[i].moveDirection == AGV_FORK_MOVE_BACKWARD){
                if(lineTheta > 0 ){
                    lineTheta -= PI;
                }
                else{
                    lineTheta += PI;
                }
            }
        }
        agvForkPathVec[i].agvHeadTheta = lineTheta;
    }

    agvForkPathVec[0].agvHeadTheta = agvForkPathVec[1].agvHeadTheta;
    return 1;
}

/*
static int32 dThetaProcess(float& dTheta){
    float theta = dTheta;
    if( theta > PI ){
        theta -= 2*PI;
    }
    else if(theta < -PI){
        theta += 2*PI;
    }

    if (theta > 1.57){
        theta = 1.57;
    }
    else if(theta < -1.57 ){
        theta = -1.57;
    }
    dTheta = theta;
    return 1;
}
*/

/*
    拐弯方向应该留在最后决定
*/
/*
static int32 setAgvForkHeadTheta4( std::vector<PathNode>& agvForkPathVec ){

    if ( agvForkPathVec.size() < 2 ){
        return 0;
    }

    float dy = agvForkPathVec[1].pt.y - agvForkPathVec[0].pt.y;
    float dx = agvForkPathVec[1].pt.x - agvForkPathVec[0].pt.x;
    float startTheta = atan2(dy, dx);

    if( agvForkPathVec[ 1 ].moveDirection == AGV_FORK_MOVE_BACKWARD ){
        if( startTheta > 0 ){
            startTheta -= PI;
        }
        else{
            startTheta += PI;
        }
    }
    agvForkPathVec[0].agvHeadTheta = startTheta;

    float bias = 20; //20mm的一个阈值，判断是不是同一个点
    for( unsigned int i = 1; i < agvForkPathVec.size(); i++){
        int turnDirection = agvForkPathVec[i].turnDirection;
        Point ptStart;
        Point ptEnd;
        float lineTheta;
        float dist = fabs(agvForkPathVec[i-1].pt.x - agvForkPathVec[i].pt.x) + fabs(agvForkPathVec[i-1].pt.y - agvForkPathVec[i].pt.y);
        if(dist < bias ){ //路径上的两个点相隔太近，视为同一个点
            lineTheta = agvForkPathVec[i-1].agvHeadTheta;
        }
        else{
            if(turnDirection == AGV_FORK_TURN_DIRECTION_LEFT || turnDirection == AGV_FORK_TURN_DIRECTION_RIGHT ){ //左拐或右拐
                float dTheta = agvForkPathVec[i].dTheta;
                dThetaProcess( dTheta );
                lineTheta = agvForkPathVec[i-1].agvHeadTheta + dTheta; //拐弯到达的点的角度，用前一点的角度和拐弯角度一起计算
                //dThetaProcess( lineTheta );
                if( lineTheta > PI ){
                    lineTheta -= 2*PI;
                }
                else if( lineTheta < -PI ){
                    lineTheta += 2*PI;
                }
            }
            else{
                ptStart = agvForkPathVec[i-1].pt;
                ptEnd = agvForkPathVec[i].pt;
                lineTheta = atan2(ptEnd.y - ptStart.y, ptEnd.x - ptStart.x); //直线的地方，直接通过直线角度来计算
                if( agvForkPathVec[i].moveDirection == AGV_FORK_MOVE_BACKWARD ){
                    if(lineTheta > 0 ){
                        lineTheta -= PI;
                    }
                    else{
                        lineTheta += PI;
                    }
                }
            }
        }
        agvForkPathVec[i].agvHeadTheta = lineTheta;
    }

    return 1;
}
*/

/*
static int32 setAgvId(std::vector<PathNode>& agvForkPathVec, uint32 agvId ){
	for (auto it = agvForkPathVec.begin(); it < agvForkPathVec.end(); it++){
		it->agvId = agvId;
	}
	return 1;
}
*/

static int32 addActionLoadGoods(const TaskInfo& tkInfo, std::vector<PathNode>& agvForkPathVec){

    float heightLiftFork = tkInfo.forkWorkType.heightLiftFork;
    float heightLiftGoods = tkInfo.forkWorkType.heightLiftGoods;
//    float heightWork = tkInfo.forkWorkType.heightWork;
    float extendDistance = tkInfo.forkWorkType.distance;

    if(agvForkPathVec.size() < 3 ){
        qDebug()<<"agvForkPath2StandardPath.cpp::addActionLiftGoods, the size of pathVec is not enought to gennerate additoinal point for get goods!";
        return -1;
    }

    size_t vecSize = agvForkPathVec.size();

    //边走边升点
    PathNode& startLiftNode = agvForkPathVec[vecSize - 3];
    startLiftNode.height = heightLiftFork;

    Point pt_1 = agvForkPathVec[vecSize-2].pt; //the last 2nd nodes
    Point pt_2 = agvForkPathVec[vecSize-1].pt; // the last nodes
    Point ptExtend_2;
/*
    //举起货叉
    PathNode pNodeAdjustTheta_1 = agvForkPathVec.back();
    pNodeAdjustTheta_1.action = 5;
    pNodeAdjustTheta_1.height = heightLiftFork;
    pNodeAdjustTheta_1.dTheta = 0;
*/
    PathNode pNodeLiftForks = agvForkPathVec.back();
    pNodeLiftForks.action = AGV_FORK_LOCK_UP;
    pNodeLiftForks.height = heightLiftFork; //lift forks
    pNodeLiftForks.dTheta = 0;

    //到达取货点
    getLineExtendPoint(&pt_1, &pt_2, extendDistance, &ptExtend_2);
    PathNode pNodeClose2Shelf = agvForkPathVec.back();
    pNodeClose2Shelf.action = 1;
    pNodeClose2Shelf.pt = ptExtend_2;
    pNodeClose2Shelf.moveDirection = AGV_FORK_MOVE_BACKWARD;
    pNodeClose2Shelf.turnDirection = AGV_FORK_TURN_NONE;
    pNodeClose2Shelf.speed = 0.3;
    pNodeClose2Shelf.endPointVelocityMode = 1;

    //作角度调整
    PathNode pNodeAdjustTheta_2 = pNodeClose2Shelf;
    pNodeAdjustTheta_2.speed = 0;
    pNodeAdjustTheta_2.action = 5;
/*
    //检查误差是否符合要求
    PathNode pNodeCheckPos = pNodeClose2Shelf;
    pNodeCheckPos.speed = 0;
    pNodeCheckPos.action = 7;
*/
    //举起或放下货物
    PathNode pNodeLiftGoods = pNodeClose2Shelf;
    pNodeLiftGoods.action = AGV_FORK_LOCK_UP;
    pNodeLiftGoods.height = heightLiftGoods;

    PathNode pNodeBack = pNodeLiftGoods;
    pNodeBack.pt = pNodeLiftForks.pt;
    pNodeBack.moveDirection = AGV_FORK_MOVE_FORWARD;
    pNodeBack.turnDirection = AGV_FORK_TURN_NONE;
    pNodeBack.action = AGV_FORK_ACTION_NONE_LOCK;
    pNodeBack.endPointVelocityMode = 2;
    pNodeBack.height = 0;
    pNodeBack.speed = 0.6;
/*
    PathNode pNodeDownForks = pNodeBack;
    pNodeDownForks.action = 6;
    pNodeDownForks.speed = 0.4;
    pNodeDownForks.height = heightWork;
*/
   // agvForkPathVec.push_back( pNodeAdjustTheta_1 );
    agvForkPathVec.push_back( pNodeLiftForks );
    agvForkPathVec.push_back( pNodeClose2Shelf );
    agvForkPathVec.push_back( pNodeAdjustTheta_2 );
    agvForkPathVec.push_back( pNodeLiftGoods );
    agvForkPathVec.push_back( pNodeBack );
    //agvForkPathVec.push_back( pNodeDownForks );
    qDebug()<<"action add succeed!\n";
    return 1;
}


int32 setWorkType(const TaskInfo& tkInfo, std::vector<PathNode>& agvForkPathVec){
    ForkWorkType forkWorkType = tkInfo.forkWorkType;
    if( forkWorkType.workType == AGV_FORK_WORK_TYPE_LOAD_GOODS){
        addActionLoadGoods(tkInfo, agvForkPathVec);

    }
    else if( forkWorkType.workType == AGV_FORK_WORK_TYPE_NONE ){
        int vecSize = agvForkPathVec.size();
        agvForkPathVec[vecSize - 1].endPointVelocityMode = 2;
        agvForkPathVec[vecSize - 1].action = 1;
        qDebug()<<"no work Type";
    }
    else{
        qDebug()<<"workType is not defined!";
        return -1;
    }
    return 1;
}


static int32 setAgvForkMapBias(std::vector<PathNode>& agvForkPathVec, PositionOffset posOffset){
    if(agvForkPathVec.size() < 2 ){
        qDebug()<<"can not deal, agvForkPath2StandardPath.cpp, setAgvForkMapBias()";
        return -1;
    }
    int vecSize = agvForkPathVec.size();
    PathNode& node_2 = agvForkPathVec[vecSize - 2];
    PathNode& node_1 = agvForkPathVec[vecSize - 1];
    node_1.pt.x += posOffset.dx * 1000;
    node_1.pt.y += posOffset.dy * 1000;
    node_2.pt.x += posOffset.dx * 1000;
    node_2.pt.y += posOffset.dy * 1000;
    return 1;
}

static int32 offsetWorkPlatRoute(const TaskInfo& tkInfo, std::vector<PathNode>& agvForkPathVec){
    int dx = tkInfo.taskOffsetInfo.posOffsetX * 1000;
    int dy = tkInfo.taskOffsetInfo.posOffsetY * 1000;
    int vecSize = agvForkPathVec.size();
    //由于每个agv的坐标原点不一，将进入工位的最后一段偏移，第一个拐弯点作为偏移的终止条件（包括拐弯结束点）
    for( int index = vecSize - 1; index > -1; --index  ){
        PathNode& node = agvForkPathVec[index];
        node.pt.x += dx;
        node.pt.y += dy;
        if( node.turnDirection == 1 || node.turnDirection == 2 ){
            break;
        }
    }
    return 1;
}

static int32 addWorkType(std::vector<PathNode>& agvForkPathVec, const TaskInfo& tkInfo) {
    ForkWorkType forkWorkType = tkInfo.forkWorkType;
    if( forkWorkType.workType == AGV_FORK_WORK_TYPE_LOAD_GOODS){
        addActionLoadGoods(tkInfo, agvForkPathVec);
    }
    else if( forkWorkType.workType == AGV_FORK_WORK_TYPE_NONE ){
        int vecSize = agvForkPathVec.size();
        if( vecSize < 1 ){
            return -1;
        }
        //offsetWorkPlatRoute(taskContentRecv, agvForkPathVec);
        agvForkPathVec[vecSize - 1].endPointVelocityMode = 1;
        agvForkPathVec[vecSize - 1].action = 1;
        qDebug()<<"no work Type";
    }
    else{
        qDebug()<<"workType is not defined!";
        return -1;
    }
    return 1;
}

int getOffsetInfo( int agvId, int benchId ){
    for( int i = 0; i < gMaxOffsetNum; i++ ){
        if( offsetInfo[i].agvId == agvId && offsetInfo[i].benchId == benchId ){
            return i;
        }
    }
    return -1;
}

int offsetRoute( std::vector<PathNode>& agvForkPathVec, const TaskInfo& tkInfo ){
    int agvId = tkInfo.agvId;
    int benchId = tkInfo.id_2;
    int index = getOffsetInfo( agvId, benchId );
    if( index < 0 ){
        return 1;       //no offset info
    }
    int offsetX = offsetInfo[index].offsetX;
    int offsetY = offsetInfo[index].offsetY;
    int vecSize = agvForkPathVec.size();
    //由于每个agv的坐标原点不一，将进入工位的最后一段偏移，第一个拐弯点作为偏移的终止条件（包括拐弯结束点）
    for( int index = vecSize - 1; index > -1; --index  ){
        PathNode& node = agvForkPathVec[index];
        node.pt.x += offsetX;
        node.pt.y += offsetY;
        if( node.turnDirection == 1 || node.turnDirection == 2 ){
            break;
        }
    }
    return 1;

}

int32 processFirstNode( std::vector<PathNode>& agvForkPathVec ){
    if( agvForkPathVec.size() < 2 ){
        return -1;
    }

    PathNode& node_0 = agvForkPathVec[0];
    PathNode& node_1 = agvForkPathVec[1];
    node_0.moveDirection = node_1.moveDirection;
    node_0.turnDirection = AGV_FORK_TURN_NONE;

    for( unsigned int i = 0; i < agvForkPathVec.size(); i++ ){
        PathNode& node = agvForkPathVec[i];
        node.nodeOrder = i;
    }
    return 1;
}

int32 convertAgvForkPath2StandardPath(const TaskInfo& tkInfo, std::vector<PathNode>& agvForkPathVec){

    setMoveMode(agvForkPathVec, tkInfo.agvForkOutType ); //����moveDirection �� turnDirection�� ���Ǳ�׼���˶�ģʽת��Ϊ��׼���˶�ģʽ
    //setNodeSpeedMode(agvForkPathVec); //����·���ϵ����ٶ�ģʽ��Բ����Ϊ���٣�����ȫ��Ϊ�������٣��յ�����
    //setVelocityMode(agvForkPathVec);
    //setActionMode(agvForkPathVec); //����action�����˶����������仯�ĵط�Ҫ��action����Ϊ1����ʹ����������

    addWorkType(agvForkPathVec, tkInfo);

    offsetRoute(agvForkPathVec, tkInfo);

    setLayerMode(agvForkPathVec);	//����ÿ������layer
    setAgvForkHeadTheta(agvForkPathVec);	 // 最理想的，车头角度该是多少度，就是多少度（没有对大于1.57度的情况进行处理）
    //setAgvForkHeadTheta4(agvForkPathVec); // 当拐弯角度小于90度时 和方法1相同的处理效果，当车头转动角度大于1.57时，会当1.57处理，对-1.57也是同样处理，没有累计偏差
    processFirstNode( agvForkPathVec );
    //setAgvId(agvForkPathVec, taskContentRecv.agvId);

	return 1;
}

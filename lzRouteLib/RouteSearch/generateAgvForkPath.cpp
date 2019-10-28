#include<vector>
#include"./generateAgvForkPath.h"
#include"./GeometryCalculate.h"


static bool isOffsetIndex( int id ){
    bool bOffsetNode = false;
    if(   id >= 103 && id <= 108  ){
        bOffsetNode = true;
    }
    else if( id == 70 || id == 95 ){
        bOffsetNode = true;
    }
    return bOffsetNode;
}

static int32 offsetNode(PathNode& node, float offsetX, float offsetY ){
    node.pt.x += offsetX * 1.000;
    node.pt.y += offsetY * 1.000;
    return 1;
}


static int32 getOffsetKeyPath( std::vector<PathNode>& agvForkPathVec, float offsetX, float offsetY ){
    bool bOffset = false;
    int bouMax = agvForkPathVec.size();
    for( int i = 0; i < bouMax; i++){ //在搜出来的一个关键节点里，找出偏移点，然后施加偏移
        bOffset = false;
        PathNode& node = agvForkPathVec[i];
        if( isOffsetIndex(node.id) ){ //偏移点
            bOffset = true;
        }
        if( bOffset ){
            offsetNode( node, offsetX, offsetY ); //施加偏移
        }
    }
    return 1;
}

static int32 keyNodePath2PathNode(const std::vector<int>& keyNodePath, std::vector<PathNode>& path ){
	    PathNode node;
        uint32 bouMax = keyNodePath.size();
        for( uint32 i = 0; i < bouMax; i++){
            uint32 id = keyNodePath[i];
            node.id = id;
            node.pt.x = keyNodes[id].pt.x;
            node.pt.y = keyNodes[ id ].pt.y;
            path.push_back( node );
        }
		return 1;
}

static int32 getNextNodeId( const int id, const int id_1, const int id_2, int& id_next ){
    KeyNode& node = keyNodes[id];
    for( uint32 j = 0; j < MAXOUT; j++ ){
        uint32 id_x = node.outNode[j];
        if( id_x && id_x != id_1 && id_x != id_2 ){
            id_next = id_x;
            return 1;
        }
    }
     return -1;
}

//this data struct can be optimized, but the amount of the calculation is not big, keep the same
static int32 getTurnInfo( int id_1, int id_2, int id_3 ){
    for( int i = 1; i < gMaxTurnNum; i++ ){
        if( turnInfo[i].id_1 == id_1  && turnInfo[i].id_2 == id_2 && turnInfo[i].id_3 == id_3 ){
            return i;
        }
    }
    //正向没找到就找反向
    for( int i = 1; i < gMaxTurnNum; i++ ){
        if( turnInfo[i].id_3 == id_1 && turnInfo[i].id_2 == id_2 && turnInfo[i].id_1 == id_3 ){
            return -i;
        }
    }
    return 0;
}

static int32 getSpeedInfo( int id_1, int id_2, int id_3 ){
    for( int i = 1; i < gMaxTurnNum; i++ ){
        if( speedInfo[i].id_1 == id_1 && speedInfo[i].id_2 == id_2 && speedInfo[i].id_3 == id_3 ){
            return i;
        }
    }
    //正向没有找到就找反向
    for( int i = 1; i < gMaxTurnNum; i++){
        for( int i = 0; i < gMaxTurnNum; i++){
            if( speedInfo[i].id_3 == id_1 && speedInfo[i].id_2 == id_2 && speedInfo[i].id_1 == id_3 ){
                return -i;
            }
        }
    }
    return 0;
}

static int32 generateSpecifiedTangentPoint(Point* pt1, Point* pt2, Point* pt3, int turnType, float turnRadius, Point* pt_c, Point* pt_tg1, Point* pt_tg2, float* dTheta){
    if( turnType == 1 ){
        generateTangentPoint( pt1, pt2, pt3, turnRadius, pt_c, pt_tg1, pt_tg2, dTheta);
    }
    else if( turnType == 2 ){
        Point extendPoint;
        float extendLen = 1.000;
        getLineExtendPoint(pt1, pt2, extendLen, &extendPoint );
        generateTangentPoint( &extendPoint, pt2, pt3, turnRadius, pt_c, pt_tg1, pt_tg2, dTheta );
    }

    else if( turnType == 3 ){
        Point extendPoint;
        float extendLen = 1.000;
        getLineExtendPoint( pt3, pt2, extendLen, &extendPoint );
        generateTangentPoint( pt1, pt2, &extendPoint, turnRadius, pt_c, pt_tg1, pt_tg2, dTheta);
    }

    return 1;
}

int getSpeed(int id_1, int id_2, int id_3, int turnDirection, float& speed_1, float& speed_2 ){
    if( turnDirection == 0 ){ //直线情况，就一个速度
        int infoIndex = getSpeedInfo(id_1, 0, id_3);
        if( infoIndex == 0 ){
            speed_1 = defaultLINEspeed;
        }
        else if( infoIndex < 0 ){
            speed_1 = speedInfo[-infoIndex].speed;
        }
        else if( infoIndex > 0 ){
            speed_1 = speedInfo[infoIndex].speed;
        }
    }
    else if( turnDirection == 1 || turnDirection == 2 || turnDirection == 3 || turnDirection == 4 ){ //第二种类型拐弯速度设置
        int infoIndex_1 = getSpeedInfo(id_1, 0, id_3);
        int infoIndex_2 = getSpeedInfo(id_1, id_2, id_3);
        if( infoIndex_1 == 0 ){
            speed_1 = defaultLINEspeed;
        }
        else if( infoIndex_1 < 0 ){
            speed_1 = speedInfo[-infoIndex_1].speed;
        }
        else if( infoIndex_1 > 0 ){
            speed_1 = speedInfo[infoIndex_1].speed;
        }

        if( infoIndex_2 == 0 ){
            speed_2 = defaultCURVEspeed;
        }
        else if( infoIndex_2 < 0 ){
             speed_2 = speedInfo[-infoIndex_2].speed;
        }
        else if( infoIndex_2 > 0 ){
             speed_2 = speedInfo[infoIndex_2].speed;
        }
    }
    return 1;
}

//各种信息，0都放默认值

static int32 doubleNodesProcess( std::vector<PathNode>& path ){
    PathNode &pNode_1 = path[0];
    PathNode &pNode_2 = path[1];
    pNode_1.moveDirection = 1;
    pNode_1.turnDirection = 0;
    pNode_1.dTheta = 0;
    pNode_1.action = 1;
    pNode_2.moveDirection = 1;
    pNode_2.turnDirection = 0;
    pNode_2.dTheta = 0;
    pNode_2.action = 1;
    getSpeed(pNode_1.id, pNode_2.id, 0, 0, pNode_2.speed, pNode_1.speed);
    return 1;
}

static int32 generateForkPathNode(std::vector<PathNode>& path, float32 turnRadius){

    if ( path.size() < 2){ //只有超过三个点，才需要刚才的处理
        return -1;
    }
    else if( path.size() == 2 ){
        doubleNodesProcess(path);
        return 1;
    }

    std::vector<PathNode> temp(path.begin(), path.end());
    PathNode node;

    path.clear();
    path.push_back(temp[0]);

    Point* pt1, *pt2, *pt3;
    Point pt_c, pt_tg1, pt_tg2;// pt_extend;
    float32 dTheta;
    int32 turnDirection = 0;

    float32 len_back_dist_2 = 0.300; //第二种拐弯方式后退的距离
    float32 len_back_dist_3 = 0.300; //第二种方式拐弯的后退距离
    float32 len_back_dist_4 = 0.300; //第四种拐弯方式后退的距离
    uint32 bouMax = temp.size() - 1;
    int moveDirection = 1;

    for (int i = 1; i < bouMax; i++){

        int id_1 = temp[i-1].id;
        int id_2 = temp[i].id;
        int id_3 = temp[i+1].id;

        pt1 = &(temp[i - 1].pt);
        pt2 = &(temp[i].pt);
        pt3 = &(temp[i + 1].pt);

        getTurnDirection(pt1, pt2, pt3, turnDirection);   //暂时规定，只可能在取货物时存在后退情形

        if( turnDirection == 0 ){
            int infoIndex = getTurnInfo( id_1, id_2, id_3 );
            if( turnInfo[ infoIndex ].turnType == 4 ){
                turnDirection = 4; //三点共线的一种特殊拐弯
            }
        }

        float speed_1 = 0.0;
        float speed_2 = 0.0;

        if ( turnDirection == 0 ){//直行
            getSpeed(id_1, 0, id_2, 0, speed_1, speed_2);
            node = temp[i];
            node.moveDirection = moveDirection;
            node.turnDirection = 0;
            node.speed = speed_1;
            node.dTheta = 0.0;
            path.push_back(node); // 设置直行的运行参数,只有一个拐弯参数，已被压入
        }
        else{//拐弯情况进行处理, //查询弯道类型
            int turnType = 0;
            float turnRadiusSpecial = 1.0;
            int infoIndex = getTurnInfo( id_1, id_2, id_3 );
            if( infoIndex == 0 ){ //表里没有查找到，就是默认值1
                turnType = 1;
                turnRadiusSpecial = turnRadius;
            }
            else if( infoIndex > 0 ){
                turnType = turnInfo[ infoIndex ].turnType;
                turnRadiusSpecial = turnInfo[ infoIndex ].radius;
            }
            else if( infoIndex < 0 ){ //正向没找到，用的是反向的信息
                turnType = 1;
                turnRadiusSpecial = turnInfo[ -infoIndex ].radius;
            }

            if( turnRadiusSpecial < 0.005 ){
                turnRadiusSpecial = turnRadius;
            }

            getSpeed(id_1, id_2, id_3, turnType, speed_1, speed_2);
            generateSpecifiedTangentPoint(pt1, pt2, pt3, turnType, turnRadiusSpecial, &pt_c, &pt_tg1, &pt_tg2, &dTheta);

            if( turnType == 1 ){ //第一种类型拐弯
                node.id = id_2;
                node.pt = pt_tg1;
                node.moveDirection = moveDirection;
                node.turnDirection = 0;
                node.dTheta = 0.0;
                node.speed = speed_1;
                path.push_back( node );

                node.pt = pt_tg2;
                node.moveDirection = moveDirection;
                node.turnDirection = turnDirection;
                node.dTheta = dTheta;
                node.speed = speed_2;
                node.ptCenterX = pt_c.x;
                node.ptCenterY = pt_c.y;
                path.push_back( node );
            }
            else if( turnType == 2 ){ //第二种类型的拐弯
                //到达折线拐弯的拐点
                node.id = temp[i].id;
                node.pt = *pt2;
                node.moveDirection = moveDirection;
                node.turnDirection = 0;
                node.dTheta = 0.0;
                node.speed = speed_1;
                path.push_back( node );

                //第一个切点
                node.id = temp[i].id;
                node.pt = pt_tg1;
                node.moveDirection = moveDirection;
                node.turnDirection = 0;
                node.dTheta = 0.0;
                node.speed = speed_1;
                node.action = 2;
                path.push_back(node);

                //为防止脱轨的延长点
                Point ptExtend;
                getLineExtendPoint( pt2, &pt_tg1, len_back_dist_2, &ptExtend);
                node.id = temp[i].id;
                node.pt = ptExtend;
                node.moveDirection = moveDirection;
                node.turnDirection = 0;
                node.dTheta = 0.0;
                node.speed = 0.2;
                node.action = 1;
                path.push_back(node);

                moveDirection = -moveDirection; //切换运动方向

                //走到第一个切点
                node.id = temp[i].id;
                node.pt = pt_tg1;
                node.moveDirection = moveDirection;
                node.turnDirection = 0;
                node.dTheta = 0.0;
                node.action = 2;
                node.speed = speed_2;
                path.push_back(node);

                //第二个切点
                node.id = temp[i].id;
                node.pt = pt_tg2;
                node.moveDirection = moveDirection;
                node.turnDirection = turnDirection * (-1); //第二类拐弯是相反类型的拐弯
                node.dTheta = dTheta;
                node.action = 2;
                node.speed = speed_2;
                node.ptCenterX = pt_c.x;
                node.ptCenterY = pt_c.y;
                path.push_back(node);

            }
            else if( turnType == 3){//第三种类型的拐弯
                //第一个切点
                node.id = temp[i].id;
                node.pt = pt_tg1;
                node.moveDirection = moveDirection;
                node.turnDirection = 0;
                node.action = 2;
                node.dTheta = 0.0;
                node.speed = speed_1;
                path.push_back(node);

                //第二个切点
                node.id = temp[i].id;
                node.pt = pt_tg2;
                node.moveDirection = moveDirection;
                node.turnDirection = turnDirection *(-1);
                node.speed = speed_2;
                node.action = 2;
                node.dTheta = dTheta;
                node.ptCenterX = pt_c.x;
                node.ptCenterY = pt_c.y;
                path.push_back(node);

                //第三个点，延长点
                float extendLen = turnRadius + len_back_dist_3;
                Point extendPoint;
                getLineExtendPoint(pt3, pt2, extendLen, &extendPoint );
                node.id = temp[i].id;
                node.pt = extendPoint;
                node.moveDirection = moveDirection;
                node.turnDirection = 0;
                node.speed = 0.2;
                node.dTheta = 0.0;
                node.action = 1;
                path.push_back(node);

                moveDirection = -moveDirection; //切换车头方向

                //到达折线拐弯的拐点
                node.id = temp[i].id;
                node.pt = *pt2;
                node.moveDirection = moveDirection;
                node.turnDirection = 0;
                node.dTheta = 0.0;
                node.action = 2;
                node.endPointVelocityMode = 1;
                node.speed = speed_2;
                path.push_back(node);
            }
            else if( turnType == 4 ){

                int nextId;
                int ret = getNextNodeId( id_2, id_1, id_3, nextId);
                if (ret != 1){
                    printf("3 points to shelf in line, and no other brach, can not deal!\n");
                    return -1;
                }

                Point ptExtend;
                PathNode nodeAid;
                nodeAid.pt.x = keyNodes[nextId].pt.x;
                nodeAid.pt.y = keyNodes[nextId].pt.y;
                getTurnDirection(pt1, pt2, &(nodeAid.pt), turnDirection);
                generateTangentPoint(pt1, pt2, &(nodeAid.pt), turnRadiusSpecial, &pt_c, &pt_tg1, &pt_tg2, &dTheta);

                //1走到一个点（开始拐弯点）
                node.id = temp[i].id;
                node.pt = pt_tg1;
                node.moveDirection = moveDirection;
                node.turnDirection = 0;
                node.dTheta = 0.0;
                node.speed = speed_1;
                node.endPointVelocityMode = 2;
                node.height = 0.0;
                node.action = 2;
                path.push_back(node);

                //2拐弯结束点
                node.id = temp[i].id;
                node.pt = pt_tg2;
                node.moveDirection = moveDirection;
                node.turnDirection = turnDirection;
                node.dTheta = dTheta;
                node.speed = speed_2;
                node.endPointVelocityMode = 2;
                node.height = 0.0;
                node.action = 2;
                node.ptCenterX = pt_c.x;
                node.ptCenterY = pt_c.y;
                path.push_back(node);

                //3到达延长点
                getLineExtendPoint(pt2, &pt_tg2, len_back_dist_4, &ptExtend);
                node.id = temp[i].id;
                node.pt = ptExtend;
                node.moveDirection = moveDirection;
                node.turnDirection = 0;
                node.dTheta = 0;
                node.speed = 0.1;
                node.endPointVelocityMode = 2;
                node.height = 0.0;
                node.action = 1;
                path.push_back(node);

                getTurnDirection(&(nodeAid.pt), pt2, pt3, turnDirection);
                generateTangentPoint(&(nodeAid.pt), pt2, pt3, turnRadiusSpecial, &pt_c, &pt_tg1, &pt_tg2, &dTheta); //获取第二次拐弯的终点

                moveDirection = -1 * moveDirection; //反向
                //4到达开始拐弯点
                node.id = temp[i].id;
                node.pt = pt_tg1;
                node.moveDirection = moveDirection;
                node.turnDirection = 0;
                node.dTheta = 0.0;
                node.speed = speed_2;
                node.endPointVelocityMode = 2;
                node.height = 0.0;
                node.action = 2;
                path.push_back(node);

                //5拐弯结束点
                node.id = temp[i].id;
                node.pt = pt_tg2;
                node.moveDirection = moveDirection;
                node.turnDirection = turnDirection;
                node.dTheta = dTheta;
                node.speed = speed_2;
                node.endPointVelocityMode = 2;
                node.height = 0.0;
                node.action = 2;
                node.ptCenterX = pt_c.x;
                node.ptCenterY = pt_c.y;
                path.push_back(node);
            }
        }
    }

    int id_1 = temp[bouMax-1].id;
    int id_2 = temp[bouMax].id;
    float speed_1 = 0.0;
    float speed_2 = 0.0;
    getSpeed(id_1, 0, id_2, 0, speed_1, speed_2);

    node = temp[bouMax]; //前進到取貨點
    node.turnDirection = 0;
    node.dTheta = 0.0;
    node.speed = speed_1;
    node.action = 1;
    node.endPointVelocityMode = 1;
    node.moveDirection = moveDirection;
    path.push_back(node);
    return 1;
}

int32 generateAgvForkPath( const TaskInfoGenerate& taskContentGenerate, const std::vector<int32>& keyNodePathVec, std::vector<PathNode>& agvForkPathVec){
	
    	int ret = keyNodePath2PathNode( keyNodePathVec,  agvForkPathVec); //将keyNodePath转化为NodePath形式的数据， 并标记拐点
        float routeOffsetX = taskContentGenerate.routeOffsetX;
        float routeOffsetY = taskContentGenerate.routeOffsetY;
        ret = getOffsetKeyPath( agvForkPathVec, routeOffsetX, routeOffsetY );
        if( ret != 1 ){
            printf("offset failed!(generateAgvForkPath.cpp, generateAgvForkPath())");
            return -1;
        }

		float32 turnRadius = taskContentGenerate.turnRadius;
        ret = generateForkPathNode( agvForkPathVec, turnRadius );

        if(ret != 1 ){
           printf("generate Standard path failed!\n");
           return ret;
        }

       return 1;
}

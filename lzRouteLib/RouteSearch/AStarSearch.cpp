
#include"./AStarSearch.h"
#include<QDebug>

/* to get the key node path via A star search method */
/* 
	get keyNode path
	@taskContent contains the task information
	@keyNodePathVec contains the path searched
	if search succeed, return 1, else return 0;
*/

static uint32 start_id;		//搜索起点的id号 
static uint32 goal_id;		//搜索目标点的id号 
static KeyNode *startNode; //指向搜索起点的指针 
static KeyNode *goalNode;  //指向搜索终点的指针 
static float32 start_x_pos;		//起点的x坐标 
static float32 start_y_pos;		//起点的y坐标 
static float32 goal_x_pos;		//目标点的x坐标 
static float32 goal_y_pos;		//目标点的y坐标 
static float32 agvHeadTheta;	//叉车车头当前角度 


struct cmpAStar{
    bool operator()( const KeyNode* node_1, const KeyNode* node_2 ){
        if( node_1->fScore == node_2->fScore ){
            return node_1->pt.y < node_2->pt.y;
        }
        else
            return node_1->fScore > node_2->fScore;
    }
};
std::priority_queue< KeyNode*, std::vector<KeyNode*>, cmpAStar > openList; //A*算法中的openList 
std::vector<KeyNode*> closeList;	//A*算法中的closeList 
std::vector<int> keyNodePath;		//搜索结果存储的容器 

/*
	the function is used to get the distance between two key nodes
	@node1 node 1
	@node2 node 2
	@dist a pointer, *dist stores the distance between node1 and node2
	return 1 if succeed 
*/ 

/*
static int getDistance( const KeyNode* node1, const KeyNode* node2,  float *dist ){
    float dx = node1 -> pt.x - node2 -> pt.x ;
    float dy = node1 -> pt.y - node2 -> pt.y;
    *dist = sqrt(dx*dx + dy*dy);
    return 1;
}
*/

static int getGDistance( const KeyNode* node1, const KeyNode* node2,  float *dist ){
    float dx = node1 -> pt.x - node2 -> pt.x ;
    float dy = node1 -> pt.y - node2 -> pt.y;
    *dist = sqrt(dx*dx + dy*dy) + node1->gScore;
    return 1;
}

static int getHDistance( const KeyNode* node1, const KeyNode* node2,  float *dist ){
    float dx = node1 -> pt.x - node2 -> pt.x ;
    float dy = node1 -> pt.y - node2 -> pt.y;
    *dist = sqrt(dx*dx + dy*dy);
    return 1;
}

/*
	the function is used to distinguish the current node is the goal node or not
	@currentNode current node
	return 1 if the current node is the goal node, else return 0
	notice:the goal node (goalNode) is a global variable
*/
static bool isGoal( const KeyNode* currentNode ){
    if( currentNode -> id == goalNode -> id )
            return 1;
        else
            return 0;
}

/*
	the function is used to update the openList variable in the A* algorithm
	@currentNode current Node
	@neighborNode neighbor Node
	the function always return 1
	notice:to know more about the function, please refer to A* algorithm
*/
/*
static int updateNeighborNodeAStar2( const KeyNode* currentNode, KeyNode* neighborNode ){
        float32 gScore;
        float32 hScore;
        getDistance( currentNode, neighborNode, &gScore ); //获取当前点到邻接点的距离
        getDistance( neighborNode, goalNode, &hScore ); //获取邻接点到目标点的距离
        float32 fScore = gScore + hScore;
        if( neighborNode->isInOpenList  ){ // if the neighbor is not in openList ?
            if( fScore < neighborNode->fScore || ( fScore == neighborNode->fScore && hScore < neighborNode->hScore) ){
                neighborNode->gScore = gScore;
                neighborNode->hScore = hScore;
                neighborNode->fScore = fScore;
                neighborNode->parrentId = currentNode->id;
            }
        }
        else{
            neighborNode->gScore = gScore;
            neighborNode->hScore = hScore;
            neighborNode->fScore = fScore;
            neighborNode->parrentId = currentNode->id;
            neighborNode->isInOpenList = 1;
            openList.push( neighborNode );
        }
        return 1;
}
*/

static int updateNeighborNodeAStar( const KeyNode* currentNode, KeyNode* neighborNode ){
        float32 gScore;
        float32 hScore;
        getGDistance( currentNode, neighborNode, &gScore ); //获取当前点到邻接点的距离
        getHDistance( neighborNode, goalNode, &hScore ); //获取邻接点到目标点的距离
        float32 fScore = gScore + hScore;
        if( neighborNode->isInOpenList  ){ // if the neighbor is not in openList ?
            if( fScore < neighborNode->fScore || ( fScore == neighborNode->fScore && hScore < neighborNode->hScore) ){
                neighborNode->gScore = gScore;
                neighborNode->hScore = hScore;
                neighborNode->fScore = fScore;
                neighborNode->parrentId = currentNode->id;
            }
        }
        else{
            neighborNode->gScore = gScore;
            neighborNode->hScore = hScore;
            neighborNode->fScore = fScore;
            neighborNode->parrentId = currentNode->id;
            neighborNode->isInOpenList = 1;
            openList.push( neighborNode );
        }
        return 1;
}

/*
	the function is used to initiate task content
	@taskContent contains detail information about the search
*/
static int initSearchTask( const TaskInfoSearch taskContent  )
{
    start_id = taskContent.id_1; //start id stands for the head of the search
    goal_id = taskContent.id_2;	//goal id stands for the tail of the search
    agvHeadTheta = taskContent.agvHeadTheta;//agvHeadtheta stands for the angle of the agv fork when searching

    startNode = &(keyNodes[ start_id ]);
    goalNode = &(keyNodes[ goal_id ]);

    start_x_pos = startNode -> pt.x;
    start_y_pos = startNode -> pt.y;
    goal_x_pos = goalNode -> pt.x;
    goal_y_pos = goalNode -> pt.y;
	
	return 1;
}

static int resetSearchStatus( ){

    while( !openList.empty() ){
        KeyNode* node = openList.top();
        node->isInOpenList = 0;
        node->parrentId = 0;
        openList.pop();
    }

    while( !closeList.empty()){
        KeyNode* node = closeList.back();
        node->isInCloseList = 0;
        node->parrentId = 0;
        closeList.pop_back();
    }

    keyNodePath.clear();
    return SUCCEED;
}

/*
	the function is used to searh a path available 
	
	gScore stores the distance from "start node" to "current node" in the search 
	hScore stores the distance from "current node" to the goal node in the search
	fScore stores the distance from "start node" to "goal node" if the current path is used
	notice: id is extremly important, little trick about the openList and closeList, when a keyNode is added in openList, set its flag( isInOpenList ) to true(1), and when a
			// keyNode is pop from the openList, set its flag to false(0), the same operate applys to closeList
			more about the algorithm refers to A* algorithm
*/

static int findKeyNodePathViaAStar( ){

    startNode->gScore = 0.0;
    getHDistance( startNode, goalNode, &(startNode->hScore) );
    startNode->fScore = startNode->gScore + startNode->hScore;
    startNode->parrentId = 0;

    startNode->isInOpenList = 1;
    openList.push( startNode );

    while( !openList.empty() ){
        KeyNode* currentNode = openList.top();
        openList.pop();
        currentNode->isInOpenList = 0;

        if( isGoal( currentNode ) ){  //currentNode is goal ?
            currentNode->isInCloseList = 1;
            closeList.push_back( currentNode );
            return SUCCEED;
        }

        //currentNode is not goal
        currentNode->isInCloseList = 1;
       // qDebug()<<"closeList: "<<currentNode->id;
        closeList.push_back( currentNode );

        if( closeList.size() > 120 ) { //key Step
            printf("the number of path node have exceeded 100, search process terminated automatically!");
            return SEARCH_NODES_NUM_EXCEED;
        }

        //check currentNode's neighbor
        for( int i = 0; i < MAXOUT; i++ ){
            int nextId = currentNode->outNode[i];
            if( nextId != 0 ){// this neighbor is accessiable ?
                KeyNode* neighborNode = &(keyNodes[nextId]);
                if( !(neighborNode->isInCloseList) ){
                    updateNeighborNodeAStar( currentNode, neighborNode );
                }
            }
        }
    }
    return SEARCH_CANNOT_FIND_GOAL;
}

/*
	the function is used to extract the path in closeList to keyNodePath
	@keyNodePath is a container to store the path
*/
static int extractPath2Vec(std::vector<int>& keyNodePath){
	std::vector<int> temp;
    //temp.reserve(100);
	KeyNode* pNode = closeList.back();
	while (pNode->parrentId){
		temp.push_back(pNode->id);
		pNode = &(keyNodes[pNode->parrentId]);
	}
	temp.push_back(pNode->id);
	keyNodePath.insert(keyNodePath.end(), temp.rbegin(), temp.rend());

	return 1;
}

/*
	the function is used to display the path having been searched
	@keyNodePath is a vector, contains the desired path
*/
int showKeyNodePath(const std::vector<int> &keyNodePath){
	uint32 bouMax = keyNodePath.size() - 1;
	for (uint32 i = 0; i < bouMax; i++){
		printf("%d -> ", keyNodePath[i]);
	}
	printf("%d\n", keyNodePath[bouMax]);
	return 1;
}

/*
	the function is used to check the task is valid or not
*/
int isValidTask(TaskInfoSearch taskContentSearch){
	uint32 id_1 = taskContentSearch.id_1;
	uint32 id_2 = taskContentSearch.id_2;
    if (id_1 < 1 || id_1 > nodesNum || id_2 < 1 || id_2 > nodesNum ){
        //printf("invalid task, id out of boundary(AStarSearch.cpp, isValidTask)");
        return SEARCH_INVALID_ID;
	}
	
    if ( keyNodes[id_1].id && keyNodes[id_2].id ){
        return SUCCEED;
	}
    return SEARCH_INVALID_ID;
}

/*
	the function is used to get a path between the start node and the goal node
	@taskContent contains the basic information of the search
	@keyNodePathVec is a container, contains the path that have been searched
*/
int getKeyNodePath( const TaskInfoSearch& taskContent, std::vector<int>& keyNodePathVec )
{
    int taskState = isValidTask(taskContent);
    if (taskState < 0){
        return taskState;
	}

    resetSearchStatus(); //reset the search status, clear the previous status of openList, closeList and keyNodePath
    initSearchTask( taskContent ); //set goal variable,like start node id and end node id , according to the task content

    uint32 searchState = findKeyNodePathViaAStar( ); //call A* algorithm to get the route
    if( searchState < 0 ){
        return searchState;
    }

    extractPath2Vec(keyNodePathVec);
    return SUCCEED;

}

/*
	//some doubts about the the algorithm:
		//if we use  priority queue, when updating an element in the openList, the queue will sorted atomatic?
        //if the problem above can not run as we expected(that is to say not sorted automatically), use re-insert to solve the problem?
*/

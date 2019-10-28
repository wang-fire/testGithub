#include"./loadMap.h"

KeyNode* keyNodes;
EDGE** edges;
uint32 nodesNum = 0;
uint32 edgesNum = 0;
TurnInfo turnInfo[ gMaxTurnNum ];
OffsetInfo offsetInfo[ gMaxOffsetNum ];
SpeedInfo speedInfo[ gMaxTurnNum ];

/*
    the function used to get the fisrt character when use "space character" as separator
    @str pointer to the string
    for example
        str = "hello world, how are you?";
        after sentence "str = getNextSpaceInString(str)" applies, then str points to "world, how are you?"
*/

char* getNextSpaceInString( char *str )
{
    while( *str == ' ' ){
        ++str;
    }
    while( *str != ' ') //skip all the none space character, then str pointer to the next first space character
        ++str;
    while( *str == ' ')//skip all the spce character, then str pointer to the next first none space character
        ++str;
    return str;
}

/*
 * the funtion used to load turnInfo from "turnInfo.txt"
*/

void initTurnInfo(){
    TurnInfo turnInitElement;
    turnInitElement.id_1 = 0;
    turnInitElement.id_2 = 0;
    turnInitElement.id_3 = 0;
    turnInitElement.turnType = 0;
    for( int i = 0; i < gMaxTurnNum; i++ ){
        turnInfo[i] = turnInitElement;
    }
}

void initSpeedInfo(){
    SpeedInfo speedInfoElement;
    speedInfoElement.id_1 = 0;
    speedInfoElement.id_2 = 0;
    speedInfoElement.id_3 = 0;
    speedInfoElement.speed = 0.0;
    for( int i = 0; i< gMaxTurnNum; i++ ){
        speedInfo[i] = speedInfoElement;
    }
}

int getTurnInfoFromFile(const char* turnInfoFileName ){
    initTurnInfo();
    FILE* fp = fopen( turnInfoFileName, "r");
    if(!fp){
        printf("open turnInfo.txt failed\n");
        return -1;
        //open turnInfo.txt failed;
    }
    int index = 0;
    char buf[256];

    while( !feof(fp) && index < gMaxTurnNum ){
           fgets(buf, 255, fp);

           if(buf[0] == '\n' || buf[0] == '#'){
               continue;
           }

           char* head = buf;
           uint32 id_1 = atoi( head );

           head = getNextSpaceInString( head );
           uint32 id_2 = atoi( head );

           head = getNextSpaceInString( head );
           uint32 id_3 = atoi( head );

           head = getNextSpaceInString( head );
           uint32 turnType = atoi( head );

           head = getNextSpaceInString( head );
           float32 radius = atof( head );

           TurnInfo& turn = turnInfo[ ++index ];
           turn.id_1 = id_1;
           turn.id_2 = id_2;
           turn.id_3 = id_3;
           turn.turnType = turnType;
           turn.radius = radius;
    }
    fclose(fp);
    return index;
}

int loadTurnInfo(const char* turnInfoFileName){
    int turnInfoNum = getTurnInfoFromFile( turnInfoFileName );
    return turnInfoNum;
}

void initOffsetInfo(){
    OffsetInfo offsetInitElement;
    offsetInitElement.agvId = 0;
    offsetInitElement.benchId = 0;
    offsetInitElement.offsetX = 0;
    offsetInitElement.offsetY = 0;
    for( int i = 0; i < gMaxOffsetNum; i++ ){
        offsetInfo[i] = offsetInitElement;
    }
}

int getOffsetInfoFromFile( const char* offsetInfoFileName ){
    initOffsetInfo();
    FILE* fp = fopen( offsetInfoFileName, "r");
    if(!fp){
        printf("open offsetInfo.txt failed\n");
        return -3;
        //open offsetInfo.txt failed;
    }

    int index = 0;
    char buf[256];
    while( !feof(fp) && index < gMaxOffsetNum ){
        fgets(buf, 255, fp);
        if( buf[0] == '\n' || buf[0] == '#' ){
            continue;
        }

        char* head = buf;
        int agvId = atoi(head);

        head = getNextSpaceInString(head);
        int32 benchId = atoi(head);

        head = getNextSpaceInString(head);
        int32 offsetX = atoi(head);

        head = getNextSpaceInString(head);
        int32 offsetY = atoi(head);

        OffsetInfo &offset = offsetInfo[ index++ ];

        offset.agvId = agvId;
        offset.benchId = benchId;
        offset.offsetX = offsetX;
        offset.offsetY = offsetY;
    }
    fclose( fp );
    return index;
}

int getSpeedInfoFromFile( const char* speedInfoFileName ){
    initSpeedInfo();
    FILE* fp = fopen(speedInfoFileName, "r");
    if(!fp){
        printf("speed info file load failed!\n");
        return -1;
    }
    char buf[256];
    SpeedInfo spInfo;
    int cnt = 0;
    while(!feof(fp)){
        fgets(buf, 255, fp);
        if( buf[0] == '#' || buf[0] == '\n'  || buf[0] == '\0'){
            continue;
        }
        char* head = buf;
        int id_1 = atoi(head);

        head = getNextSpaceInString(head);
        int id_2 = atoi(head);

        head = getNextSpaceInString(head);
        int id_3 = atoi(head);

        head = getNextSpaceInString(head);
        float speed = atof(head);

        spInfo.id_1 = id_1;
        spInfo.id_2 = id_2;
        spInfo.id_3 = id_3;
        spInfo.speed = speed;

        speedInfo[++cnt] = spInfo;
    }
    fclose(fp);
    return cnt;
}

int loadOffsetInfo(const char* offsetInfoFileName ){
    int offsetInfoNum = getOffsetInfoFromFile( offsetInfoFileName );
    return offsetInfoNum;
}

/*
    the function is used to convert edge's topological information to "inNodes" and "outNodes" in keyNode struct
    notice: a node's in-degree must be less than MAXIN, and out-degree must be less than MAXOUT
*/
static int convertEdgeRelation2KeyNodeInfo()
{
    for( uint32 currentId = 1; currentId <= nodesNum; currentId++ ){
       for(  int j = 0; j < MAXOUT; j++ ){
            int neighBorId = edges[currentId][j].id;
            if(neighBorId){
                for( int m = 0; m < MAXOUT; m++){ //�1�7�1�7outNode�1�7�1�7�1�7�1�7�0�5�1�7�1�7�1�7�0�6�1�7�˄1�7�0�0�1�7�1�7�1�7�1�7�1�7currentId�1�7�1�7�1�7�1�7�1�7�1�9�1�7�1�7�1�7�1�7�0�7�1�7
                    if( keyNodes[currentId].outNode[m] == 0 ){
                        keyNodes[currentId].outNode[m] = neighBorId;
                        break;
                    }
                }
                for( int m = 0; m < MAXIN; m++ ){
                   if(keyNodes[neighBorId].inNode[m] == 0 ){
                       keyNodes[neighBorId].inNode[m] = currentId;
                        break;
                   }
                }
            }
       }
    }
    return SUCCEED;
}

/*
    the function is used to check the content in buf are same with str or not
    @buf stores the data to match
    @str stores the content to match
    same return 1, else return 0
*/

static int isStringBegin( char buf[], const char* str ){
    uint32 bouMax = strlen(str);
    for( uint32 i = 0; i < bouMax; i++){
        if( buf[i] != str[i] )
            return 0;
    }
    return 1;
}

/*
    the function is used to get the number of nodes and edges in the Map
    @fileName is the Map file
    @nodesNum is the number of nodes in the Map
    @edgesNum is the number of edges in the Map
*/
static int getMapInfo(const char *fileName, uint32 &nodesNum, uint32 &edgesNum, uint32& maxNodeId )
{
    nodesNum = 0;
    edgesNum = 0;
    char buf[256];
    FILE* fp = fopen( fileName, "r" );
    if( !fp ){
        printf("file open failed!(getMapInfo)\n");
        exit( -1 );
    }

    while( !feof(fp) ){
        fgets( buf, 255, fp );
        if( isStringBegin(buf, "KeyNode Locations:") == 1 ){
            printf("start reads map Info!\n");
            break;
        }
    }

    while( !feof(fp) ){
        fgets( buf, 255, fp );
        if( buf[0] == ' ' || buf[0] == '#' || buf[0] == '\n')
            continue;
        if( isStringBegin( buf, "Edge Relation Begin:" )){
            printf("KeyNode Loacation reads complete!\n");
            break;
        }
        else{
            int id = atoi(buf);
            if(id > maxNodeId){
                maxNodeId = id;
            }
            ++nodesNum;
        }

    }

    while( !feof(fp)){
        fgets( buf, 255, fp );
        if( buf[0] == ' ' || buf[0] == '#' || buf[0] == '\n')
            continue;
        if( isStringBegin(buf, "Map Info End")){
            printf("Edge Info reads complete!\n");
            break;
        }
        else
            ++edgesNum;
    }
    fclose(fp);
    if(nodesNum < 1 || edgesNum < 1 )
        return -1;
    return 1;
}


/*
    the function is used to free the memory whcih used to store the map information
    notice:
        this function is called, when the search program is terminated
*/
static int freeMapMemory(){
    if( edges ){
        delete []edges[0];
       // delete edges;
        edges = NULL;
    }
    if( keyNodes ){
      delete[] keyNodes;
      keyNodes = NULL;
    }
    return 1;
}

/*
    the function is used to allocate memory for Map
    @nodesNum is the number of node in the Map
    @edgesNum is the number of edge in the Map
    return -1 if the memory allocates failed, else return 1
*/

static int allocateMapMemory( uint32 nodesNum ){

    edges = new EDGE*[nodesNum];
    if (!edges){
        printf("allocating memory for edges pointer failed!(loadMap.cpp, allocateMapMeomry)\n");
        return -1;
    }
    else{
        for(int i = 0; i <= nodesNum; i++){
            edges[i] = NULL;
        }
    }

    edges[0] = new EDGE[nodesNum*MAXOUT]; //edges start from 1
    if ( !edges[0] ){
        printf("allocating memory for edges failed!(loadMap.cpp, allocateMapMeomry)\n");
        delete []edges;
        return -1;
    }

    for (uint32 i = 1; i <= nodesNum; i++){
        edges[i] = edges[i - 1] + MAXIN; //a way to allocate memory for 2 dimensional array
    }

    keyNodes = new KeyNode[nodesNum];
    if(!keyNodes){ //fail to allocate memory for the key nodes
        delete []edges[0];//delete all the memory allocated for the edges
        edges[0] = NULL;
        delete []edges; //delete the edges pointer
        edges = NULL;
        printf("keyNodes memory allocate failed!(loadMap.cpp, allocateMapMemory)");
        return -1;
    }

    return SUCCEED;
}


/*
    the function is used to load the Map which is used for seaching
    return 1 if the Map load succeed, else return -1
*/

int cLoadMap( const char* fileName, const char* turnInfoFileName, const char* offsetInfoFileName, const char* speedInfoFileName ){

    cCloseMap();
    uint32 maxNodeId = 0;
    //get the number of nodes and edges in the Map
    int ret = getMapInfo( fileName, nodesNum, edgesNum, maxNodeId);
    if(ret < 0 ){
        printf("file info gets failed!(loadMap.cpp, loadMap)\n");
        return FILE_LOAD_ROUTE_MAP_FAILED;
    }
    printf("mapInfo, node num:%d, edge num:%d\n", nodesNum, edgesNum);

    //allocate memory to store the Map topological relationship
    ret = allocateMapMemory(maxNodeId + 1);
    if(ret < 0 ){
        printf("loadMap.cpp, cLoadMap(), memory allocate failed!\n");
        return MEMORY_ALLOCATE_FAILED;
    }
    else{
        if( rSPrintDebugInfo ){
            printf("allocate memory for map succeed!\n");
        }
    }

    char buf[256];
    FILE* fp = fopen( fileName, "r");
    if (!fp){
        printf("file open failed!(loadMap.cpp, loadMap)\n");
        return FILE_OPEN_FAILED_NOT_EXIST;
    }

    //locates start part of the KeyNode in the Map
    while(!feof(fp) && (isStringBegin(buf, "KeyNode Locations:") != 1)){
        fgets( buf, 255, fp );
    }

    int recordsNum = nodesNum;
    while( !feof(fp) && recordsNum ){
        fgets(buf, 255, fp);
        if( buf[0] == ' ' || buf[0] == '#' || buf[0] == '\n' || buf[0] == 'K')
            continue;

        char* head = buf;
        uint32 id = atoi(head);

        head =getNextSpaceInString(head);
        float32 x = (float32)atof( head );

        head = getNextSpaceInString( head );
        float32 y = (float32)atof( head );

        head = getNextSpaceInString( head );
        int32 layer = (int32)atoi(head);

        keyNodes[id].id = id;
        keyNodes[id].pt.x = x;
        keyNodes[id].pt.y = y;
        keyNodes[id].layer = layer;

        --recordsNum;
    }

    //locating the start part of edges in the Map
    while( !feof(fp) && (isStringBegin(buf, "Edge Relation Begin:") != 1) ){
        fgets( buf, 255, fp );
    }

    recordsNum = edgesNum;
    while( !feof(fp) && recordsNum ){
        fgets( buf, 255, fp );
        if( buf[0] == ' ' || buf[0] == '#' || buf[0] == '\n' )
            continue;
        char *head = buf;
        int id_1 = atoi(head);
        head = getNextSpaceInString(head);
        int id_2 = atoi(head);
        head = getNextSpaceInString(head);
        int turnDir = atoi(head);
        for( int j = 0; j < MAXOUT; j++ ){
            if(edges[id_1][j].id == 0 ){
                EDGE* edge = &(edges[id_1][j]);
                edge -> id = id_2;
                edge ->turnDir = turnDir;
                break;
            }
        }
        --recordsNum;
    }

    fclose(fp);
    nodesNum = maxNodeId;
    ret = convertEdgeRelation2KeyNodeInfo();  //convert edge data struct into key node data struct
    if( ret < 0 ){
        printf("convert failed!\n");
        return DATA_CONVERT_FAILED;
    }
    if( rSPrintDebugInfo ){
        printf("convert complete!\n");
    }

    //load turn info
    ret =  getTurnInfoFromFile( turnInfoFileName );
    if( ret < 0 ){
        return FILE_LOAD_TURN_INFO_FAILED;
    }

    //load offset info
    ret = getOffsetInfoFromFile( offsetInfoFileName );
    if( ret < 0 ){
        return FILE_LOAD_OFFSET_INFO_FAILED;
    }

    //load speed info
    ret = getSpeedInfoFromFile( speedInfoFileName );
    if(ret < 0){
       return FILE_LOAD_SPEED_INFO_FAILED;
    }

    return SUCCEED;
}

int cCloseMap(){
    freeMapMemory();
    return MEMORY_FREE_SUCCEED;
}

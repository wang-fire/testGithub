#include "./moveScript.h"
#include "./agvForkParam.h"
#include <QDebug>

typedef struct MoveInfo_{
    int cmdType;
    float attr_1;
    float attr_2;
    float attr_3;
    MoveInfo_():cmdType(0), attr_1(0.0), attr_2(0.0), attr_3(0.0){}
}MovInfo;

static bool stringMatch( char* str1, const char* str2 ){
    if( strlen(str1) < strlen(str2) ){
        return false;
    }
    int size = strlen(str2);
    for( int i = 0; i < size; i++ ){
        if( *(str1 + i) != *(str2 + i) ){
            return false;
        }
    }
    return true;
}

static char* convert2StandardBuf(char* buf ){
    int index = 0;
    while(buf[index] && buf[index] != '\n' ){
        if( !('0' <= buf[index] && buf[index] <= '9') && buf[index] != '-' && buf[index] != '.'){
           buf[ index ] = ' ';
        }
        index++;
    }
    return buf;
}
static char* getNextSpaceInString( char *str )
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

static int32 loadScriptInfo( const char* fileName, MovInfo movInfo[] ){

    FILE* fp =fopen(fileName, "r");
    if( !fp ){
        //qDebug()<<"load Script failed!";
        return FILE_OPEN_FAILED_NOT_EXIST; //
    }
    char buf[256];
    char* head = NULL;
    MovInfo mov;
    int cnt = 0;

    while(!feof(fp)){
        fgets(buf, 255, fp);
        if( buf[0] == '\n' || buf[0] == '#' ){
            continue;
        }
        int cmdType = 0;
        head = buf;
        if( stringMatch(head, "MOV") ){
            cmdType = 1;
        }
        else if( stringMatch(head, "ROT")){
            cmdType = 5;
        }
        else if( stringMatch(head, "LFT") ){
            cmdType = 6;
        }
        else{
            continue;
        }

        head = convert2StandardBuf( buf );
        float attr_1 = atof( head );
        head = getNextSpaceInString( buf );
        float attr_2 = atof( head );

        mov.cmdType = cmdType;
        mov.attr_1 = attr_1;
        mov.attr_2 = attr_2;

        movInfo[cnt++] = mov;

        if( cnt > 20 ){
            return SEARCH_SCRIPT_TOO_MANY_ACTIONS;
        }
    }
    fclose(fp);
    return cnt;
}

static int32 getLineExtendNode(const PathNode& nodeStart, PathNode& nodeExtend, float dist){
    nodeExtend = nodeStart;
    float agvHeadTheta = nodeStart.agvHeadTheta;
    float posX = nodeStart.pt.x;
    float posY = nodeStart.pt.y;
    float dx = dist * cos(agvHeadTheta);
    float dy = dist * sin(agvHeadTheta);
    nodeExtend.pt.x = posX + dx;
    nodeExtend.pt.y = posY + dy;
    return 1;
}

int32 movScriptProcess( std::vector<PathNode>& agvPath, const char* fileName ){
    MovInfo movInfo[20];
    int movNum = loadScriptInfo( fileName, movInfo );
    if( movNum < 0 ){
       // qDebug()<<"load script info failed!";
        return SEARCH_LOAD_SCRIPT_FAILED;
    }

    PathNode node;
    int nodeAddedNum = 0;
    for(int i = 0 ; i < movNum; i++ ){
        MovInfo& mov = movInfo[i];
        node = agvPath.back();
        if(mov.cmdType == 1 ){ //直线行走
            float dist = mov.attr_1;
            float speed = mov.attr_2;
            getLineExtendNode(agvPath.back(), node, dist);
            if( dist < 0 ){
                node.moveDirection = AGV_FORK_MOVE_BACKWARD;
            }
            else{
                node.moveDirection = AGV_FORK_MOVE_FORWARD;
            }
            node.endPointVelocityMode = 1;
            node.turnDirection = AGV_FORK_TURN_NONE;
            node.forkControlType = 1;
            node.action = 1;
            node.speed = speed;
            node.dTheta = 0;

            agvPath.push_back(node);
        }
        else if( mov.cmdType == 5 ){ //旋转
            //float dTheta = mov.attr_1;
            //float rotateWay = mov.attr_2;
            node = agvPath.back();
            node.action = 5;
            agvPath.push_back(node);
        }
        else if(mov.cmdType == 6 ){ //举升
            float liftHeight = mov.attr_1;
            //float liftSpeed = mov.attr_2;
            node.action = 6;
            node.height = liftHeight;
            agvPath.push_back(node);
            int lastNodeIndex = (int)(mov.attr_2 + 0.01);
            if( lastNodeIndex > 0 ){
                for( int i = 1; i < lastNodeIndex + 1; i++){
                     int index = agvPath.size() - 1 - i;
                     if( index >= 0 ){
                         agvPath[index].height = liftHeight;
                     }
                }
            }
        }
        else{
            --nodeAddedNum;
        }
        ++nodeAddedNum;
    }
  //  qDebug()<<nodeAddedNum<<"action was added!";
    return SUCCEED;
}

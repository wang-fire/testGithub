#ifndef ERRORTYPEDEF_H
#define ERRORTYPEDEF_H

const int SUCCEED = 1;

const int DB_CREATE_TABLE_SUCCEED = 13;
const int SUCCEED_DB_NO_UPDATE = 2;
const int SUCCEED_DB_HAVE_UPDATE = 3;
const int ROUTE_MAP_NOT_CHANGED = 4;
const int ROUTE_MAP_UPDATE_SUCCEED = 5;
const int DB_OPEN_SUCCEED = 14;
const int DB_NO_SEARCH_TASK = 15;
const int SEARCH_HVAE_NO_TASK = 16;
const int MEMORY_FREE_SUCCEED = 17;
const int DB_GET_TASK_SUCCEED = 18;
const int DB_NO_ROUTE_TO_SAVE = 19;
const int DB_WRITE_ROUTE_SUCCEED = 20;
const int DB_SUCCEED_UPDATE_STATE = 21;
const int SEARCH_SUCCEED_NO_TASK = 22;
const int SEARCH_SUCCEED_GET_TASK = 23;

const int FAILED = -1;

const int OPEN_DB_FAILED = -2;
const int CLOSE_DB_FAILED = -3;
const int WRITE_DB_FAILED = -4;
const int READ_DB_FAILED = -5;

const int SEARCH_FAILED_WRONG_ID = -6;
const int SEARCH_FAILED_NON_EXIST_ID = -7;
const int SEARCH_FAILED_NOT_FIND = -8;

const int FILE_OPEN_FAILED_NOT_EXIST = -9;
const int FILE_OPEN_EMPTY_FILE = -10;
const int FILE_OPEN_FAILED_CREATE = -11;

const int WRITE_FILE_FAILED = -12;
const int DB_CREATE_TABLE_FAILED = -13;
const int DB_OPEN_FAILED = -14;
const int DB_READ_TABLE_FAILED = -15;
const int DB_CREATE_CONNECT_FAILED = -16;
const int DB_QUERY_FAILED = -17;
const int DB_INIT_TABLE_MODEL_FAILED = -18;
const int DB_DELETE_CONTENT_FAILED = -27;

const int SEARCH_ID_NOT_VALID = -19;

const int FILE_LOAD_ROUTE_MAP_FAILED = -20;
const int FILE_LOAD_TURN_INFO_FAILED = -21;
const int FILE_LOAD_SPEED_INFO_FAILED = -22;
const int FILE_LOAD_OFFSET_INFO_FAILED = -23;
const int FILE_LOAD_MOVE_INFO_FAILED = -24;
const int MEMORY_ALLOCATE_FAILED = -25;

const int DATA_CONVERT_FAILED = -26;

const int SEARCH_KEY_PATH_SEARCH_FAILED = -27;
const int SEARCH_DETAIL_PATH_GENERATE_FAILED = -28;

const int SEARCH_INVALID_ID = -29;
const int SEARCH_NODES_NUM_EXCEED = -30;
const int SEARCH_CANNOT_FIND_GOAL = -31;
const int DB_FAILED_UPDATE_STATE = -32;

const int SEARCH_FAILED_QUERY_TASK = -33;
const int SEARCH_LOAD_SCRIPT_FAILED = -34;
const int SEARCH_SCRIPT_TOO_MANY_ACTIONS = -35;



#endif // ERRORTYPEDEF_H

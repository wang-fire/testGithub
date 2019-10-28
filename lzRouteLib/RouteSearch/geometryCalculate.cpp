#include"./geometryCalculate.h"

/*
    该函数求解思路：
        1.直线方程用ax+by+c = 0来表示，避免斜率无穷大的问题
        2.计算pt1-pt2直线的方程中的a1与b1
        3.计算pt2-pt3直线的方程中的a2与b2
        4.计算pt1-pt2 与 pt2-pt3两条直线的角平分线的a3与b3
        5.圆心在角平分线上，圆心到角的一条边(pt1-pt2 或 pt2-pt3)的距离等于圆的半径r， 对这两个约束进行求解（使用行列式），即可得到圆心坐标
    注意：
        1.最后一步利用了点（圆心）到直线的距离方程 以及 点（圆心）在直线上的方程， 两个方程联立求解
        2.求出的圆心有两个（一个位于角内，一个位于角外，关于角尖对称），有一个需要排除，排除方法：
            pt_center-pt2构成的直线与角的一条边的夹角必定小于90度（不符合要求的大于90°），向量乘积大于0
        3.除以len的目的是，将直线的a与b进行归一化处理
*/
int getCircleCenter( const Point* pt1, const Point* pt2, const Point* pt3,  const float turnRadius, Point* pt_center ){

        float32 dx21 = pt1->x - pt2->x;
        float32 dy21 = pt1->y - pt2->y;
        float32 dx23 = pt3->x - pt2->x;
        float32 dy23 = pt3->y - pt2->y;
        float32 len1 = sqrt( dx21 * dx21 + dy21 * dy21 );
        float32 len2 = sqrt( dx23 * dx23 + dy23 * dy23 );
        float32 a1 = dx21 / len1;
        float32 b1 = dy21 / len1;
        float32 a2 = dx23 / len2;
        float32 b2 = dy23 / len2;
        float32 a3 = (float32)(( a1 + a2 ) * 0.5);
        float32 b3 = (float32)(( b1 + b2 ) * 0.5);

		float32 r = turnRadius;

        float32 len3 = sqrt(a3 * a3 + b3 * b3 );
        a3 = a3 / len3;
        b3 = b3 / len3;

        float32 temp;
        temp = a1;
        a1 = -b1;
        b1 = temp;

        temp = a2;
        a2 = -b2;
        b2 = temp;

        temp = a3;
        a3 = -b3;
        b3 = temp;

        float32 c1 = -( a1 * pt2 -> x + b1 * pt2 -> y );
        //float c2 = -( a2 * pt2 -> x + b2 * pt2 -> y );
        float32 c3 = -( a3 * pt2 -> x + b3 * pt2 -> y );

        float32 det =  (float32)1.0 / ( a1 * b3 - b1 * a3 );
        float32 x0 = 0.0;
        float32 y0 = 0.0;

        for( uint32 i = 0; i < 2; i++ ){
            if( i == 1 )
                r = -r;
            float32 t = c1 + r;
            x0 = -( t * b3 - b1 * c3 ) * det;
            y0 = -( -t * a3 + a1 * c3 ) * det;
            float32 dx20 = x0 - pt2->x;
            float32 dy20 = y0 - pt2->y;
            float32 sign = dx20 * dx21 + dy20 * dy21;
            if( sign > 0 )
                break;
        }

        pt_center -> x = x0;
        pt_center -> y = y0;

        return 1;

}

/*
*函数求解思路：
    *1.计算直线（ax + by + c = 0）的 a 与 b
    *2.联立两个约束方程进行求解（使用行列式的方法）：
        *2.1 切点到圆心构成的向量 与 直线的方向向量 乘积 为0
        *2.2 切点在直线上
*/

int getTangentPoint( const Point* pt1, const Point* pt2, const Point* pt_center, Point* pt_tangent ){

        float32 a = -( pt2->y - pt1->y );
        float32 b = pt2->x - pt1->x;
        float32 c = -( a * pt1 -> x + b * pt1 -> y );
        float32 det = (float32) 1.0 / ( a * a + b * b );
        float32 t = ( a * pt_center->y - b * pt_center->x );
        pt_tangent -> x = ( -a * c - b * t )*det;
        pt_tangent -> y = ( -b * c + a * t )*det;
        return 1;
}

/*
 * 函数求解思路：
    *计算直线两个长度的 dx 与 dy
    *若dx 与 dy 中的任意一个为 0， 则是水平线或垂直线， 直接在水平或垂直方向上加（减）一个 len
    *若dx 与 dy均不为 0， 则根据斜率，在dx与dy方向等比例增长
*/
int getLineExtendPoint( const Point* pt1, const Point* pt2, const float len, Point* pt_ext ){

        float32 dx = pt2->x - pt1->x;
        float32 dy = pt2->y - pt1->y;

        float dist = sqrt(dx * dx + dy * dy);
        if( dist == 0 ){
            *pt_ext = *pt2;
            printf("geometryCalculate.cpp, getLineExtendPoint(), extend error!");
            return -1;
        }

        float lenRatio = len / dist;
        pt_ext->x = pt2->x + lenRatio * dx;
        pt_ext->y = pt2->y + lenRatio * dy;

		return 1;
}

/*
 * 函数求解思路：
 *  计算圆心与两个端点的夹角，用圆弧终止点的角度减去圆弧起始端点的角度
 * 注意：这里的pt_end 与 pt_start 都是通过前面程序确定的，能够保证对应的是一个小于PI的圆弧
*/
int getArcTheta(const Point* pt_center, const Point* pt_start, const Point* pt_end, float *theta_ptr)
{
    float32 theta_start = atan2( pt_start->y - pt_center->y, pt_start->x - pt_center->x );
    float32 theta_end = atan2( pt_end->y - pt_center->y, pt_end->x - pt_center->x );
    *theta_ptr = theta_end - theta_start;
	return 1;
}

/*
 * 函数求解思路：
 *  该函数通过整个几个子函数完成了一个完整的逻辑，实现了功能：自动拐弯点的计算
*/
int generateTangentPoint( const Point* pt1, const Point* pt2, const Point* pt3, const float r, Point* pt_center, Point* pt_tangent1, Point* pt_tangent2, float* theta_ptr ){
        getCircleCenter( pt1, pt2, pt3, r, pt_center );
        getTangentPoint( pt1, pt2, pt_center, pt_tangent1 );
        getTangentPoint( pt2, pt3, pt_center, pt_tangent2 );
        getArcTheta(pt_center, pt_tangent1, pt_tangent2, theta_ptr);
        return 1;
}

int getTurnDirection( const Point *pt1, const Point *pt2, const Point *pt3, int& turnDir ){
        float32 theta1 = atan2(  pt2->y - pt1->y , pt2->x - pt1->x );
        float32 theta2 = atan2(  pt3->y - pt2->y , pt3->x - pt2->x );
        float biasThetaToLine = 8.0 / 180.0 * PI; //控制直线或拐弯的阈值夹角
        /*
        if( fabs( theta1 - theta2 ) < biasThetaToLine || fabs(fabs( theta1 - theta2) - 2 *PI) < biasThetaToLine ){ //如果两条直线夹角小于8°，则判断为在一条直线上
            turnDir =  0;
            return 0;
        }
        */
        float dTheta = theta2 - theta1;
        if( dTheta < -PI ){
            dTheta += 2*PI;
        }
        else if( dTheta > PI ){
            dTheta -= 2*PI;
        }

        if( -biasThetaToLine < dTheta && dTheta <  biasThetaToLine ){
            turnDir = 0;
        }
        else if( dTheta > 0 ){
            turnDir = 1;
        }
        else if( dTheta < 0 ){
            turnDir = -1;
        }
        else{
            turnDir = -3;
            return -1;
        }
        return 1;
        /*
        if( theta2 - theta1 > 0 && theta2 - theta1 < PI ){
            turnDir = 1;
            return 1;
        }
        else if( theta2 - theta1 < 0 && theta2 - theta1 > -PI ){
            turnDir = -1;
            return -1;
        }
        else if( theta1 < 0  && theta2 >= 0 && theta2 - (2*PI + theta1) < 0 && theta2 - (2*PI+theta1) > -PI ){
            turnDir = -1;
            return -1;
        }
        else if( theta1 >= 0 && theta2 < 0 && 2*PI + theta2 - theta1 > 0 && 2*PI + theta2 - theta1 < PI){
            turnDir = 1;
            return 1;
        }
        else{
            return -3;
        }
        */
}

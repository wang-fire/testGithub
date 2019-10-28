#ifndef GEOMERYCALCULATE_H
#define GEOMERYCALCULATE_H

#include"./common.h"

//该文件主要提供一些api用于对几何方面的关系进行计算，如圆弧的切点，直线的延长点



/*
 * 前提： 直线AB与BC有一个公共交点B
 * 函数功能： 当给定A（pt1）， B（pt2）， C（pt3）与圆的半径radius后， 该函数用于求解角ABC的内切圆的圆心pt_center
 * @pt1 第一个点的坐标
 * @pt2 第二个点的坐标
 * @pt3 第三个点的坐标
 * @radius 所求内切圆的半径
 * @pt_center 所要求获取的内切圆的圆心
 * 注意：1.该函数对“角ABC”的角度位于0-180度（不包含0和180度）的情形都适用，但是没有做具体的检查，如是否为0°或180°
*/
int getCircleCenter( const Point* pt1, const Point* pt2, const Point* pt3, const float radius, Point* pt_center );



/*
*函数功能： 已知一条直线的两个端点 pt1 与 pt2， 圆心pt_center， 求切点pt_tangent1
* @pt1 线的端点1
* @pt2 线的端点2
* @pt_center 圆心
* @pt_tangent 切点（输出）
* 注意：该圆心是通过叉车拐弯半径计算出的（在函数getCircleCenter中），所以计算出的切点到圆心的距离在误差范围内是等于转弯半径的，
*       但是没有作具体的检查（切点到圆心的距离是否等于转弯半径），“正确”的使用是不会出问题的。
*/
int getTangentPoint(const Point* pt1, const Point* pt2, const Point* pt_center, Point* pt_tangent1 );


/*
*函数功能： 已知一个直线的两个端点， 在 pt2处 按 原直线方向 延长一个len的距离， 得到延长点 pt_ext
* @pt1 直线的端点1
* @pt2 直线的端点2
* @len 要延长的距离
* @pt_ext 延长后的点
*/


int getLineExtendPoint( const Point* pt1, const Point* pt2, const float len, Point* pt_ext );

/*
 * 函数功能：已知圆弧的圆心pt_center， 起点与终点， 计算圆弧的角度(小于PI的那部分)
 * @pt_center 圆心
 * @pt_start 起点
 * @pt_end 终点
 * @theta_ptr 圆弧角度（小于PI）
*/
int getArcTheta( const Point* pt_center, const Point* pt_start, const Point* pt_end, float *theta_ptr);

/*
 * 函数功能： 1.给定一个角的三个点 ABC， 圆的半径，
 *          2.求圆心坐标， 两个切点， 以及角所对应的内切圆弧的角度
 * @pt1 角ABC的端点A
 * @pt2 角ABC的端点B
 * @pt3 角ABC的端点C
 * @r 内切圆的半径
 * @pt_center 圆心
 * @pt_tangent1 圆的一个切点
 * @pt_tangent2 圆的另一个切点
 * @theta_ptr 内切圆弧对应的角度
 * 注意：
 *      1.该函数用于计算自动拐弯的点
 *      2.该函数通过调用其它几个子函数来实习（ getCircleCenter, getTangentPoint, getArcTheta);
*/
int generateTangentPoint( const Point* pt1, const Point* pt2, const Point* pt3, const float r, Point* pt_center, Point* pt_tangent1, Point* pt_tangent2, float* theta_ptr );

/*
*   函数功能：根据三个给定的点，确定这三个点的拐弯方向
*   @pt1 角ABC的端点A
*   @pt2 角ABC的端点B
*   @pt3 角ABC的端点C
*  例如:
*   turnDir = 0, AB与BC在同一条直线上
*   turnDir = -1, AB通过顺时针旋转可以与BC同向(旋转角度必须小于180°)
*   turnDir = 1, AB通过逆时针旋转可以与BC同向(旋转角度必须小于180°)
*/

int getTurnDirection( const Point *pt1, const Point *pt2, const Point *pt3, int& turnDir );

#endif // GEOMERYCALCULATE_H

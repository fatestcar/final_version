//
// Created by 王瑞华 on 2018/10/8.
//

//
// Created by 王瑞华 on 2018/10/5.
//
#include "GPIOlib.h"
using namespace GPIO;

const double d_max=15.0;//前轮中心在中轴线右侧的最大距离 单位：cm
const double v=5.0;    //恒定速度  单位：cm/s
const double l_max=45.0;//最大转动角度
const double kp=0.5;
const double ki=0.0;
const double kd=0.1;

double currentError=0.0;//当前时刻偏差
double lastError=0.0;//上一时刻偏差
double sigmaError=0.0;//累计偏差

//x 为前轮中心的位置。在中轴线上为0，在中轴线左侧为负，在中轴线右侧为正。
//返回轮胎转动角度，正数向右侧转，负数向左侧转。
double getOutput(double x);

double PID_Controller(double pos);

double getOutput(double x){
    double res=0.0;
    if(x<(-1)*d_max){
        //若超过左侧距离最大值，则记为最大值
        x=(-1)*d_max;
    }else if(x>d_max){
        //若超过右侧距离最大值，则记为最大值
        x=d_max;
    }
    //将PID算法对于距离的调整输出转化为对于转动角度的输出
    res=x*(l_max/d_max);
    return res;
}


/*
 * PID 控制器
 *
 * 离散化的PID公式如下
 * U(x)=kp*error(x)+ki*sigma(error(x),0,T)+kd*(error(x)-error(x-1))
 * 做中轴线的垂线，垂线在轴线左侧的部分是负坐标，在轴线右侧的部分是正坐标，坐标单位是cm
 * 记点x为前轮中心当前的位置，U(x)为PID计算出的应该修正的距离，调用getOutput方法变为车轮转角输出
 *
 * 传入的pos为由视觉模块测定的前轮中心当前的位置
 */
double PID_Controller(double pos){
    //设定对前轮中心位置的期望值为0，即前轮中心的位置应该在中轴线上
    double error=0-pos; //期望值与实际值的偏差，为预期调节量
    lastError=currentError;
    currentError=error;
    sigmaError=sigmaError+error;
    double Ux=kp*error+ki*sigmaError+kd*(currentError-lastError);
    double output=getOutput(Ux);
    //在这里输出转动角度
    turnTo(int(output));
    delay(1000)

    return output;//返回要输出的角度
}
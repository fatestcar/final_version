//
// Created by 刘瑷玮 on 2018/10/18.
//

#include <iostream>
#include <cmath>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "Detector.h"
#include "GPIOlib.h"
#include <thread>
#include <stdio.h>
#include <string>
#define DIST_MAX 1000000
#define DIST_MIN -1000000
using namespace std;
using namespace cv;
using namespace GPIO;

const double d_max=15.0;//前轮中心在中轴线右侧的最大距离 单位：cm
const double v=5.0;    //恒定速度  单位：cm/s
const double l_max=60.0;//最大转动角度
const double kp=0.6;
const double ki=0.0;
const double kd=0.15;

double currentError=0.0;//当前时刻偏差
double lastError=0.0;//上一时刻偏差
double sigmaError=0.0;//累计偏差


double getOutput(double x);

double PID_Controller(double pos);


const double z = 50; // 摄像头的高度，需要测量

const double width_ = 50; //跑道的宽度，需要测量

bool isRight = true;

bool isFirst = true;

double max_d = 5;

double last_value = 0;

double offset = 0;

//mutex m;

double getDistance(double k1, double k2){
    double rightDistance = abs(z/k2);
    double leftDistance  = abs(z/k1);
    rightDistance = width_*rightDistance/(rightDistance+leftDistance);
    return rightDistance;
}

int main() {
    int road_horizon = 250;
    Detector detector(road_horizon);
    KalmanFilter kf(16, 8, 0);
    Mat state (16, 1, CV_32FC1);
    Mat processNoise(16, 1, CV_32F);
    Mat measurement = Mat::zeros(8, 1, CV_32F);
    kf.transitionMatrix = (Mat_<float>(16, 16) <<
                                               1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,
            0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,
            0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,
            0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,
            0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,
            0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,
            0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,
            0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,
            0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,
            0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,
            0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,
            0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1
    );
    setIdentity(kf.measurementMatrix);
    setIdentity(kf.processNoiseCov, Scalar::all(1e-5));
    setIdentity(kf.measurementNoiseCov, Scalar::all(1e-3));
    setIdentity(kf.errorCovPost, Scalar::all(1));

    String video_path = "./VID.mp4";
    VideoCapture capture(0);

    int frame_width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    int frame_height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);

    // Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file.
    VideoWriter video("outcpp.avi",CV_FOURCC('M','J','P','G'),10, Size(frame_width,frame_height));

    bool first = false;

    init();
    controlLeft(FORWARD,5);
    controlRight(FORWARD,5);

    while(capture.isOpened()){
        Mat frame;
        capture.read(frame);
        Lanes l= detector.detect(frame);
        if(!first) {
            kf.statePost = (Mat_<float>(16, 1) <<
                                               (float)l.getLeft()[0],
                    (float)l.getLeft()[1],
                    (float)l.getLeft()[2],
                    (float)l.getLeft()[3],
                    (float)l.getRight()[0],
                    (float)l.getRight()[1],
                    (float)l.getRight()[2],
                    (float)l.getRight()[3],
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0);
            first = true;
        }
        state = kf.predict();
        if(!l.isEmpty()) {
          //  line(frame, Point(state.at<float>(0), state.at<float>(1)), Point(state.at<float>(2), state.at<float>(3)), Scalar(0, 0, 255), 3);
          //  line(frame, Point(state.at<float>(4), state.at<float>(5)), Point(state.at<float>(6), state.at<float>(7)), Scalar(0, 0, 255), 3);
        }

        double k1 = (state.at<float>(3) -  state.at<float>(1))/(state.at<float>(2) - state.at<float>(0));
        double k2 = (state.at<float>(7) -  state.at<float>(5))/(state.at<float>(6) - state.at<float>(4));


        if(l.isRightEmpty() && l.isLeftEmpty()){
            if(isRight){
                offset = 20;
             //   cout << 20 <<endl;
            }else{
                offset = -20;
            //    cout << -20 << endl;
            }
        }else if(l.isLeftEmpty()){
            isRight = true;
            offset = 20;
           // cout << "left empty" << endl;
        }else if(l.isRightEmpty()){
            isRight = false;
            offset = -20;
            //cout << "right empty" << endl;
        }else{
            offset = 25 - getDistance(k1,k2);
            if(offset > 0) isRight = true;
            else isRight = false;
            if(isFirst){
                last_value = offset;
                isFirst = false;
            }
//            if(abs(last_value - offset) > max_d){
//               // cout << "exceed" << endl;
//                offset = last_value;
//            }else{
//                last_value = offset;
//            }
        }
       // cout << offset << endl;
        double output =  PID_Controller(offset);
        char a[16];
        char b[16];
        sprintf(a,"%f", offset);
        sprintf(b,"%f", output);
        putText(frame, a, cvPoint(100,100),
                FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
        putText(frame, b, cvPoint(100,150),
                FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
        video.write(frame);
        measurement.at<float>(0) = l.getLeft()[0];
        measurement.at<float>(1) = l.getLeft()[1];
        measurement.at<float>(2) = l.getLeft()[2];
        measurement.at<float>(3) = l.getLeft()[3];
        measurement.at<float>(4) = l.getRight()[0];
        measurement.at<float>(5) = l.getRight()[1];
        measurement.at<float>(6) = l.getRight()[2];
        measurement.at<float>(7) = l.getRight()[3];
        kf.correct(measurement);
        imshow("img", frame);
        if(!waitKey(1))
            break;
    }
    return 0;
}

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
    res=x*2;// lmax/dmax
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
	cout <<Ux <<endl;
    int output=(int)getOutput(Ux);
	cout <<output <<endl;
    //在这里输出转动角度
    if(output>=15){
        output=15;
        controlLeft(FORWARD,6);
        controlRight(FORWARD,6);
        
    }else if(output<=-15){
        output=-15;
        controlLeft(FORWARD,6);
        controlRight(FORWARD,6);
    }
    turnTo(output);

    //根据当前车的位置范围控制车速，避免在急转弯时车速过快
    /*
    if(pos<-10||pos>10){
        controlLeft(FORWARD,4);
        controlRight(FORWARD,4);
    }else if(pos<-5||pos>5){
        controlLeft(FORWARD,4);
        controlRight(FORWARD,4);
    }else{
        controlLeft(FORWARD,5);
        controlRight(FORWARD,5);
    }
    */
//    delay(1000);
    return output;//返回要输出的角度
}
#include <iostream>
#include <cmath>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "Detector.h"
#define DIST_MAX 1000000
#define DIST_MIN -1000000
using namespace std;
using namespace cv;

const double z = 50; // 摄像头的高度，需要测量

const double width_ = 50; //跑道的宽度，需要测量

bool isRight = true;

bool isFirst = true;

double max_d = 5;

double last_value = 0;


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
    setIdentity(kf.processNoiseCov, Scalar::all(1e-2));
    setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(kf.errorCovPost, Scalar::all(1));

    String video_path = "./VID.mp4";
    VideoCapture capture(0);

    bool first = false;


    while(capture.isOpened()){
        Mat frame;
        capture.read(frame);
        Lanes l= detector.detect(frame);
//        if(!first) {
//            kf.statePost = (Mat_<float>(16, 1) <<
//                    (float)l.getLeft()[0],
//                    (float)l.getLeft()[1],
//                    (float)l.getLeft()[2],
//                    (float)l.getLeft()[3],
//                    (float)l.getRight()[0],
//                    (float)l.getRight()[1],
//                    (float)l.getRight()[2],
//                    (float)l.getRight()[3],
//                    0.0,
//                    0.0,
//                    0.0,
//                    0.0,
//                    0.0,
//                    0.0,
//                    0.0,
//                    0.0);
//
//            first = true;
//        }
//        state = kf.predict();

        if(!l.isEmpty()) {
            line(frame, Point((float)l.getLeft()[0], (float)l.getLeft()[1]), Point((float)l.getLeft()[2], (float)l.getLeft()[3]), Scalar(0, 0, 255), 3);
            line(frame, Point((float)l.getRight()[0], (float)l.getRight()[1]), Point((float)l.getRight()[2], (float)l.getRight()[3]), Scalar(0, 0, 255), 3);
        }

        double k1 = ((float)l.getLeft()[3] -   (float)l.getLeft()[1])/(Point((float)l.getLeft()[2] - (float)l.getLeft()[0]);
        double k2 = ((float)l.getRight()[3] -  (float)l.getRight()[1])/((float)l.getRight()[2] - (float)l.getRight()[0]);

//        cout << k1 << " " << k2 <<endl;
        double offset = 0;
        if(l.isRightEmpty() && l.isLeftEmpty()){
            if(isRight){
                offset = 20;
                cout << 20 <<endl;
            }else{
                offset = -20;
                cout << -20 << endl;
            }
        }else if(l.isLeftEmpty()){
            isRight = true;
            offset = 20;
            cout << "left empty" << endl;
        }else if(l.isRightEmpty()){
            isRight = false;
            offset = -20;
            cout << "right empty" << endl;
        }else{
            offset = 25 - getDistance(k1,k2);
            if(offset > 0) isRight = true;
            else isRight = false;
            if(isFirst){
                last_value = offset;
                isFirst = false;
            }
            cout << last_value<<" " << offset << endl;
            if(abs(last_value - offset) > max_d){
                cout << "exceed" << endl;
                offset = last_value;
            }else{
                last_value = offset;
            }
        }

        cout << offset << endl;

//        measurement.at<float>(0) = l.getLeft()[0];
//        measurement.at<float>(1) = l.getLeft()[1];
//        measurement.at<float>(2) = l.getLeft()[2];
//        measurement.at<float>(3) = l.getLeft()[3];
//        measurement.at<float>(4) = l.getRight()[0];
//        measurement.at<float>(5) = l.getRight()[1];
//        measurement.at<float>(6) = l.getRight()[2];
//        measurement.at<float>(7) = l.getRight()[3];
//        kf.correct(measurement);
//        cout<<state<<endl;

        imshow("img", frame);
        if(!waitKey(1))
            break;
    }

    return 0;
}

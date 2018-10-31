
#ifndef FASTESTCAR_LANE_H
#define FASTESTCAR_LANE_H
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
using namespace std;
using namespace cv;

class Lanes {
public:
    Lanes(const Vec4i &left, const Vec4i &right):lane_left(left), lane_right(right){}
    Lanes(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4);
    Lanes(){};
    Vec4i getLeft();
    Vec4i getRight();
    bool isLeftEmpty();
    bool isRightEmpty();
    bool isEmpty();
private:
    Vec4i lane_left;
    Vec4i lane_right;

};


#endif //FASTESTCAR_LANE_H

#include "Detector.h"

Detector::Detector(int road_horizon) {
    this->road_horizon = road_horizon;
    this->vote = 50;
    this->roi_theta = 0.3;
}


Lanes Detector::detect(Mat frame) {
    Mat grey;
    // 二值化
    cvtColor(frame, grey, COLOR_RGB2GRAY);
    // 自适应，降低明暗影响
    adaptiveThreshold(grey, grey, 255, CV_ADAPTIVE_THRESH_MEAN_C,
                      THRESH_BINARY_INV, 11, 1);

    // 腐蚀膨胀去除斑点
    Mat kernel;
    kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(grey, grey, MORPH_OPEN, kernel);
    kernel = getStructuringElement(MORPH_RECT, Size(7, 7));
    morphologyEx(grey, grey, MORPH_CLOSE, kernel);

    kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(grey, grey, MORPH_OPEN, kernel);
    kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(grey, grey, MORPH_CLOSE, kernel);

    // 高斯模糊
    GaussianBlur(grey, grey, Size(3, 3), 2.0);
    // 霍夫概率
    Mat roi = grey(Range(road_horizon, frame.rows), Range(0, frame.cols));
    Mat contours;
    Canny(roi, contours, 60, 130);
    vector<Vec4i> lines;
    HoughLinesP(contours, lines, 1, M_PI / 180, 50, 40, 150);
    Vec4i left;
    Vec4i right;
    double left_dist = DIST_MIN;
    double right_dist = DIST_MAX;

    Mat img_cut = frame(Range(road_horizon, frame.rows), Range(0, frame.cols));

    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i l = lines[i];
        l[1] += road_horizon;
        l[3] += road_horizon;

        double theta = abs(atan(1.0 * (l[3] - l[1]) / (l[2] - l[0])));
        if (theta > 0.2 && theta < 1.4) {
            double dist = dist_cross_base(l[0], l[1], l[2], l[3], frame.cols);
            if (0 > dist and dist > left_dist and atan(1.0 * (l[3] - l[1]) / (l[2] - l[0])) < 0) {
                left = scale_line(l[0], l[1], l[2], l[3], frame.cols, frame.rows);
                left_dist = dist;
            } else if (0 < dist and dist < right_dist and atan(1.0 * (l[3] - l[1]) / (l[2] - l[0])) > 0) {
                right = scale_line(l[0], l[1], l[2], l[3], frame.cols, frame.rows);
                right_dist = dist;
            }
        }
    }

    Lanes lanes = Lanes(left, right);
    return lanes;
}

double Detector::dist_cross_base(int x1, int y1, int x2, int y2, int width) {
    if (x2 == x1)
        return (width * 0.5) - x1;
    double k = (y2 - y1) * 1.0 / (x2 - x1);
    double b = y1 - k * x1;
    return (width * 0.5) - (-b / k);
}

Vec4i Detector::scale_line(int x1, int y1, int x2, int y2, int width, int height) {
    double k = (y2 - y1) * 1.0 / (x2 - x1);
    double b = y1 - k * x1;
    double temp1 = b;  //左
    double temp2 = (0 - b) * 1.0 / k;  //上
    double temp3 = k * width + b;  //右
    double temp4 = (height - b) * 1.0 / k;  //下
    if(temp4 >= 0 && temp4 <= width && temp3 >=0 && temp3 <= height) {
        return Vec4i(temp4, height, width, temp3);
    } else if(temp2 >=0 && temp2<=width && temp3 >=0 && temp3<=height){
        return Vec4i(temp2, 0, width, temp3);
    } else if(temp1 >=0 && temp1 <= height && temp4 >= 0 && temp4 <= width) {
        return Vec4i(0, temp1, temp4, height);
    } else if(temp1 >= 0 && temp1 <= height && temp2 >=0 && temp2<=width) {
        return Vec4i(0, temp1, temp2, 0);
    } else if(temp2 >= 0 && temp2<=width && temp4 >= 0 && temp4 <= width) {
        return Vec4i(temp2 , 0, temp4, height);
    } else if(temp1 >= 0 && temp1<= height && temp3 >= 0 && temp3 <= height) {
        return Vec4i(0 , temp1, width, temp3);
    }

    return Vec4i();
}

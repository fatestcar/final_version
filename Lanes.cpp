#include "Lanes.h"


Vec4i Lanes::getLeft() {
    return lane_left;
}

Vec4i Lanes::getRight() {
    return lane_right;
}




bool Lanes::isEmpty() {
    return isLeftEmpty() && isRightEmpty();
}

bool Lanes::isLeftEmpty() {
    return lane_left.val[0] == 0 && lane_left.val[1] == 0 && lane_left.val[2] == 0 && lane_left.val[3] == 0;
}

bool Lanes::isRightEmpty() {
    return lane_right.val[0] == 0 && lane_right.val[1] == 0 && lane_right.val[2] == 0 && lane_right.val[3] == 0;
}

Lanes::Lanes(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4) {
    lane_left = Vec4i(x1, y1, x2, y2);
    lane_right = Vec4i(x3, y3, x4, y4);
}

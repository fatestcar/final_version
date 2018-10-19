//
// Created by 刘瑷玮 on 2018/10/11.
//


#include "GPIOlib.h"
#include<iostream>
#include <thread>
#include <chrono>



int GPIO::init(){
    std::cout<< "init" << std::endl;
    return 0;
}

int GPIO::controlLeft(int direction,int speed){
    std::cout<< "controlLeft" << std::endl;
    return 0;
}
int GPIO::controlRight(int direction,int speed){
    std::cout<< "controlRight" << std::endl;
    return 0;
}
int GPIO::stopLeft(){
    std::cout<< "stopLeft" << std::endl;
    return 0;
}
int GPIO::stopRight(){
    std::cout<< "stopRight" << std::endl;
    return 0;
}

int GPIO::resetCounter(){
    std::cout<< "resetCounter" << std::endl;
    return 0;
}
void GPIO::getCounter(int *countLeft,int *countRight){
    std::cout<< "getCounter" << std::endl;
}

int GPIO::turnTo(int angle){
    std::cout<< "turnTo  " << angle << std::endl;
    return 0;
}

void GPIO::delay(int milliseconds){
    std::cout<< "delay" << std::endl;
//    std::this_thread::sleep_for(std::chrono::seconds(1));
}
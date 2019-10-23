#include <iostream>
#include "../include/PositioningSystem.hpp"
#include "../include/UM8E.hpp"
#include "../include/OptiTrack.hpp"

int main() {
    std::cout << "Hello Easy C++ project!" << std::endl;

    PositioningSystem* myPosSystem = new UM8E();
    myPosSystem->getPosition();

    MotionCapture* myMoCap = new OptiTrack();
    myMoCap->getAttitude();


}
//
// Created by eric on 2/19/17.
//

#ifndef HALLWAY_NAVIGATOR_IROBOT_H
#define HALLWAY_NAVIGATOR_IROBOT_H

#include <math>

class iRobot {

public:
    iRobot() {}
    enum Mode { wait, run, touch, reverse, collide };

private:
    const double reverseDelay = 2150.f/1000.f;
    const double touchDelay = reverseDelay / 4.0f;
    const double turnSpeed = M_PI / reverseDelay;
    const double reverseInterval = 20.f;
    const double noiseInterval = 5.f;
    const double speedMax = 330.f;
    const double angularOffset = M_PI/60.f;
    Mode currentMode = wait;
    double currentTime = 0.f;
};

#endif //HALLWAY_NAVIGATOR_IROBOT_H

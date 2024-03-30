#include "MyServo.h"


#ifndef LEG_H
#define LEG_H

class Leg {
public:
    Leg(MyServo& kneeServo, MyServo& ankleServo); 
    void moveLegToPosition(float kneePosition, float anklePosition); 
    void testLeg(); 
    void moveLegToPositionSmooth( float kneeEnd, float ankleEnd, float steps, float delayBetweenSteps);
    MyServo& knee; // Reference to the "knee" servo object
    MyServo& ankle; // Reference to the "ankle" servo object
private:
    float lastKnee;
    float lastAnkle;
};

#endif 
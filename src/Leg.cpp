#include "Leg.h"

Leg::Leg(MyServo& kneeServo, MyServo& ankleServo)
    : knee(kneeServo), ankle(ankleServo) {
    
}

void Leg::moveLegToPosition(float kneepos, float anklepos) {
   
    knee.setAngle(kneepos);
    ankle.setAngle(anklepos);
}
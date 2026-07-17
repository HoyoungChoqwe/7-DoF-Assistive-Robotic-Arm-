#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

#define IK_DOF 6

typedef struct {
    float x;
    float y;
    float z;
} IkVec3;

typedef struct {
    int base;
    int arm;
    int wrist;
    int top;
    int middle;
    int lower;
} IkServoAngles;

int Kinematics_SolveXYZ(IkVec3 target, IkServoAngles *angles, IkVec3 *actual);
void Kinematics_CurrentXYZ(IkVec3 *actual);
void Kinematics_FKAngles(const IkServoAngles *angles, IkVec3 *actual);
void Kinematics_ApplyAngles(const IkServoAngles *angles);

#endif

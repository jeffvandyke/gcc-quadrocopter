#ifndef
#define KALMAN_H


struct quadState_t {
    int xPosition, yPosition, zPosition;
    int xVelocity, yVelocity, zVelocity;
    int xAcceleration, yAcceleration, zAcceleration;

    int xAngle, yAngle, zAngle;
    int xRotation, yRotation, zRotation; }

#endif
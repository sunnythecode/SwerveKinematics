#pragma once
#define PI 3.14159265359
#include <cmath>

class Rotation2d
{
private:
    double radians;


public:

    Rotation2d() {
        radians = 0.0;

    }
    
    Rotation2d(double angleRadians)
    {
        radians = angleRadians;
    }

    // Returns a new Rotation2d object that is the result of adding two rotations.
    Rotation2d operator+(const Rotation2d &other) const
    {
        double sumRadians = radians + other.radians;
        return Rotation2d(sumRadians);
    }

    // Returns a new Rotation2d object that is the inverse of the current rotation.
    Rotation2d inverse() const
    {
        double inverseRadians = -radians;
        return Rotation2d(inverseRadians);
    }

    // Converts the rotation to degrees.
    double getDegrees() const
    {
        return radians * 180.0 / PI;
    }

    double getRadians() const
    {
        return radians;
    }


};

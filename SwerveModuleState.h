
#include "Rotation2d.h"

class SwerveModuleState {
    private:
        double speed_;
        double angleRadians_;

    public:
        SwerveModuleState(double speedMPS, double angleRadians) {
            speed_ = speedMPS;
            angleRadians_ = angleRadians;
        }

        SwerveModuleState(double speedMPS, Rotation2d rotRadians) {
            speed_ = speedMPS;
            angleRadians_ = rotRadians.getRadians();
        }
        SwerveModuleState() {
            speed_ = 0;
            angleRadians_ = 0;
        }
        Rotation2d getRot2d() {
            return Rotation2d(angleRadians_);
        }
        double getSpeedMPS() {
            return speed_;
        }





};
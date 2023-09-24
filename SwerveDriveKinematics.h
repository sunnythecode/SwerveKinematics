#include <EigenCore.h>
#include "Translation2d.h"
#include "Rotation2d.h"
#include "ChassisSpeeds.h"

class SwerveDriveKinematics
{
    public:

    SwerveDriveKinematics(const std::vector<Translation2d>& wheelsMeters) {
        if (wheelsMeters.size() < 2) {
            throw std::invalid_argument("A swerve drive requires at least two modules");
        }
        m_numModules = wheelsMeters.size();
        m_modules = wheelsMeters;
        m_inverseKinematics.resize(m_numModules * 2, 3);
        m_rotations.resize(m_numModules);

        for (int i = 0; i < m_numModules; i++) {
            
            m_inverseKinematics.block(i * 2, 0, 2, 3) <<
                1.0, 0.0, -m_modules.at(i).y(),
                0.0, 1.0, m_modules.at(i).x();
            //Atan2 should return radians
            m_rotations.at(i) = Rotation2d(std::atan2(m_modules[i].y(), m_modules[i].x()));
        }

        m_forwardKinematics = m_inverseKinematics.completeOrthogonalDecomposition().pseudoInverse();
    }

    std::vector<SwerveModuleState> toSwerveModuleStates(ChassisSpeeds desiredSpeeds) {

        Eigen::Vector3d chassisSpeedsVector;
        chassisSpeedsVector << 0.0, desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond, desiredSpeeds.omegaRadiansPerSecond;

        Eigen::VectorXd moduleStatesVector = m_inverseKinematics * chassisSpeedsVector;

        std::vector<SwerveModuleState> moduleStates(m_numModules);

        for (int i = 0; i < m_numModules; i++) {
            double x = moduleStatesVector(i * 2);
            double y = moduleStatesVector(i * 2 + 1);

            double speed = std::hypot(x, y);
            Rotation2d angle = Rotation2d(std::atan2(y, x));

            moduleStates[i] = SwerveModuleState(speed, angle);
        }

        return moduleStates;
    }

    

    private:
        int m_numModules;
        frc::Vectord<4> m_modules;
        frc::Matrixd m_inverseKinematics;
        std::vector<Rotation2d> m_rotations;
        frc::Matrixd< m_forwardKinematics;


        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> inverseKinematics;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> forwardKinematics;
    


};
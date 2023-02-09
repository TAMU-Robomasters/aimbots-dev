#include "gimbal.hpp"

#include <drivers.hpp>
#include <utils/common_types.hpp>

using std::vector, std::complex;


static inline float wrappedEncoderValueToRadians(int64_t encoderValue) {
    return (M_TWOPI * static_cast<float>(encoderValue)) / DJIMotor::ENC_RESOLUTION;
}

namespace src::Gimbal {

    GimbalSubsystem::GimbalSubsystem(src::Drivers * drivers)
        : tap::control::Subsystem(drivers),
          drivers(drivers),
          yawMotor(drivers,
                   YAW_MOTOR_ID,
                   GIMBAL_BUS,
                   YAW_DIRECTION,
                   "Yaw Motor"),
          pitchMotor(drivers,
                     PITCH_MOTOR_ID,
                     GIMBAL_BUS,
                     PITCH_DIRECTION,
                     "Pitch Motor"),
          currentFieldRelativeYawAngle(0.0f, 0.0f, M_TWOPI),
          currentChassisRelativeYawAngle(0.0f, 0.0f, M_TWOPI),
          currentChassisRelativePitchAngle(0.0f, 0.0f, M_TWOPI),
          targetChassisRelativeYawAngle(modm::toRadian(YAW_START_ANGLE)),
          targetChassisRelativePitchAngle(modm::toRadian(PITCH_START_ANGLE)) {

          }

    void GimbalSubsystem::initialize() {
        drivers->cvCommunicator.setGimbalSubsystem(this);
        drivers->enemyDataConverter.setGimbalSubsystem(this);

        yawMotor.initialize();
        yawMotor.setDesiredOutput(0);

        pitchMotor.initialize();
        pitchMotor.setDesiredOutput(0);
    }

    float yawChassisRelativeDisplay = 0.0f;
    float yawFieldRelativeDisplay = 0.0f;
    float pitchChassisRelativeDisplay = 0.0f;

    float pitchOutputDisplay = 0.0f;
    float yawOutputDisplay = 0.0f;

// This is ugly, but I'm just doing this for simplicity
#ifdef TARGET_HERO
    static bool isStartYawSet = false;
    static int64_t heroStartYawUnwrappedEncoder = 0;
#endif

    float currentYawAngleDisplay = 0.0f;
    float currentPitchAngleDisplay = 0.0f;

    void GimbalSubsystem::refresh() {
        if (yawMotor.isMotorOnline()) {
            // Update subsystem state to stay up-to-date with reality
            uint16_t currentYawEncoderPosition = yawMotor.getEncoderWrapped();
            currentChassisRelativeYawAngle.setValue(wrappedEncoderValueToRadians(currentYawEncoderPosition));

#ifdef TARGET_HERO
            // This code just assumes that we're starting at our
            // YAW_START_ANGLE when the robot gets turned on, and
            // then we just apply the delta from the starting encoder
            // to that after we apply the gear ratio transformation.

            int64_t unwrappedEncoder = yawMotor.getEncoderUnwrapped();

            if (!isStartYawSet) {
                isStartYawSet = true;
                heroStartYawUnwrappedEncoder = unwrappedEncoder;
            }

            float rawDelta = unwrappedEncoder - heroStartYawUnwrappedEncoder;
            float transformedDelta = rawDelta * GIMBAL_YAW_GEAR_RATIO;
            float angle = modm::toRadian(YAW_START_ANGLE) + (transformedDelta * (M_TWOPI / DJIMotor::ENC_RESOLUTION));
            currentChassisRelativeYawAngle.setValue(angle);

            float startEncoderDelta = drivers->remote.keyPressed(Remote::Key::X) - drivers->remote.keyPressed(Remote::Key::Z);
            heroStartYawUnwrappedEncoder += startEncoderDelta;
#endif

// FIXME: Verify that these plus and minus signs work out...
#ifdef TARGET_SENTRY
            currentFieldRelativeYawAngle.setValue(currentChassisRelativeYawAngle.getValue());
#else
            currentFieldRelativeYawAngle.setValue(currentChassisRelativeYawAngle.getValue() + drivers->fieldRelativeInformant.getChassisYaw() - modm::toRadian(YAW_START_ANGLE));
#endif

            currentYawAngleDisplay = modm::toDegree(currentChassisRelativeYawAngle.getValue());
            currentPitchAngleDisplay = modm::toDegree(currentChassisRelativePitchAngle.getValue());

            // Flush whatever our current output is to the motors
            yawMotor.setDesiredOutput(desiredYawMotorOutput);

            ////////////////
            // DEBUG VARS //
            ////////////////
            yawChassisRelativeDisplay = modm::toDegree(currentChassisRelativeYawAngle.getValue());
            yawFieldRelativeDisplay = modm::toDegree(currentFieldRelativeYawAngle.getValue());
            yawOutputDisplay = desiredYawMotorOutput;
        }

        if (pitchMotor.isMotorOnline()) {
            // Update subsystem state to stay up-to-date with reality
            uint16_t currentPitchEncoderPosition = pitchMotor.getEncoderWrapped();
            currentChassisRelativePitchAngle.setValue(wrappedEncoderValueToRadians(currentPitchEncoderPosition));

            pitchChassisRelativeDisplay = modm::toDegree(currentChassisRelativePitchAngle.getValue());

            pitchOutputDisplay = desiredPitchMotorOutput;

            // Flush whatever our current output is to the motors
            pitchMotor.setDesiredOutput(desiredPitchMotorOutput);
        }
    }

    void GimbalSubsystem::setYawMotorOutput(float output) {
        // This is limited since the datatype of the parameter to the function
        // `DJIMotor::setDesiredOutput()` is an int16_t. So, to make sure that
        // it isn't overflowed, we limit it within +/- 30,000, which is just
        // the max and min of an int16_t (int16_t has a range of about +/- 32,000).
        desiredYawMotorOutput = tap::algorithms::limitVal(output, -GM6020_MAX_OUTPUT, GM6020_MAX_OUTPUT);
        
    }

    void GimbalSubsystem::setPitchMotorOutput(float output) {
        // This is limited since the datatype of the parameter to the function
        // `DJIMotor::setDesiredOutput()` is an int16_t. So, to make sure that
        // it isn't overflowed, we limit it within +/- 30,000, which is just
        // the max and min of an int16_t (int16_t has a range of about +/- 32,000).
        desiredPitchMotorOutput = tap::algorithms::limitVal(output, -GM6020_MAX_OUTPUT, GM6020_MAX_OUTPUT);
    }

    float GimbalSubsystem::getCurrentYawAngleFromChassisCenter(AngleUnit unit) const {
        return tap::algorithms::ContiguousFloat(
                   (unit == AngleUnit::Degrees) ? modm::toDegree(currentChassisRelativeYawAngle.getValue() - modm::toRadian(YAW_START_ANGLE)) : (currentChassisRelativeYawAngle.getValue() - modm::toRadian(YAW_START_ANGLE)),
                   (unit == AngleUnit::Degrees) ? -180.0f : -M_PI,
                   (unit == AngleUnit::Degrees) ? 180.0f : M_PI)
            .getValue();
    }

    float GimbalSubsystem::getCurrentPitchAngleFromChassisCenter(AngleUnit unit) const {
        return tap::algorithms::ContiguousFloat(
                   (unit == AngleUnit::Degrees) ? modm::toDegree(currentChassisRelativePitchAngle.getValue() - modm::toRadian(PITCH_START_ANGLE)) : (currentChassisRelativePitchAngle.getValue() - modm::toRadian(PITCH_START_ANGLE)),
                   (unit == AngleUnit::Degrees) ? -180.0f : -M_PI,
                   (unit == AngleUnit::Degrees) ? 180.0f : M_PI)
            .getValue();
    }

    GimbalSubsystem::aimAngles GimbalSubsystem::getAimAngles(enemyTimedData data) {
        using RefSerialRxData = tap::communication::serial::RefSerialData::Rx;

        //whitespace!!!

        //get enemy position, velocity, acceleration
        double px = data.position[X_AXIS][0];
        double py = data.position[Y_AXIS][0];
        double pz = data.position[Z_AXIS][0];

        double vx = data.velocity[X_AXIS][0];
        double vy = data.velocity[Y_AXIS][0];
        double vz = data.velocity[Z_AXIS][0];

        double ax = data.acceleration[X_AXIS][0];
        double ay = data.acceleration[Y_AXIS][0];
        double az = data.acceleration[Z_AXIS][0];

        ax = 0;
        ay = 0;
        az = 0;

        vx = 0;
        vy = 0;
        vz = 0;

        double L = GIMBAL_BARREL_LENGTH; //Barrel Length Constant goes here
        double v0 = 30; //Shooter Velocity Constant found below

        //This was all copied from run_shooter_command.cpp
        //Reads the shooter connected to the ref system and pulls the current speed limit
        /*auto refSysRobotTurretData = drivers->refSerial.getRobotData().turret;
        auto launcherID = refSysRobotTurretData.launchMechanismID;
        switch (launcherID) {  // gets launcher ID from ref serial, sets speed limit accordingly
                           // #if defined(TARGET_STANDARD) || defined(TARGET_SENTRY)
            case RefSerialRxData::MechanismID::TURRET_17MM_1: {
                v0 = refSysRobotTurretData.barrelSpeedLimit17ID1;
                break;
            }
            case RefSerialRxData::MechanismID::TURRET_17MM_2: {
                v0 = refSysRobotTurretData.barrelSpeedLimit17ID2;
                break;
            }
            // #endif
            case RefSerialRxData::MechanismID::TURRET_42MM: {
                v0 = refSysRobotTurretData.barrelSpeedLimit42;
                break;
            }
            default:
                break;
        }*/

        //Need to manually find the calcs for these.
        //Created with G as 9.8m/s
        bulletDropCoEff = {(((double)0.25)*ax*ax) + (((double)0.25)*ay*ay) + (((double)0.25)*az*az) + (((double)4.9)*az) + ((double)24.01) //t^4
            ,(vx*ax) + (vy*ay) + (vz*az) + (((double)9.8)*vz) // t^3
            ,(vx*vx) + (vy*vy) + (vz*vz) + (ax*px) + (ay*py) + (az*pz) - (v0*v0) + (((double)9.8)*pz) //t^2
            ,(2*vx*px) + (2*vy*py) + (2*vz*pz) - (2*L*v0) //t
            ,(px*px) + (py*py) + (pz*pz) - (L*L) //1
            };

        double time = 0;
        time = find_root(bulletDropCoEff);

        //float pitch = 0; //phi +up
        //float yaw = 0; //theta +ccw

        aimAngles a;

        a.pitch = asin((pz+vz*time+((double)0.5)*time*time*(az+((double)9.8))) / (L+(v0*time)));

        a.yaw = acos((py+vy*time+((double)0.5)*time*time*ay) / (cos(a.pitch)*(L+(v0*time))));

        return a;
    }

    GimbalSubsystem::aimAngles GimbalSubsystem::aimAtPoint(float x, float y, float z) {
        aimAngles a;

        a.pitch = atan2(z, sqrt((x*x) + (y*y)));
        a.yaw = atan2(x,y);

        return a;
    }

}  // namespace src::Gimbal

package org.firstinspires.ftc.teamcode.Auto;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Created by 21maffetone on 2/25/18.
 *
 * Class for containing all "Actions": methods that involve multiple subsystems. These
 * tend to operate iteratively and thus must hold up the thread while they operate. These
 * mainly
 */

public class Actions {

    private static RobotHardware mRobot;

    // ONLY TO BE CALLED ONCE ROBOT IS INITIALIZED
    public static void init(RobotHardware robot) {
        // Store the instance of the robot as to use it outside the currently running
        // OpMod that calls any of these methods
        mRobot = robot;
    }

    // Action for turning to a desired angle using the integrated IMU in the REV expansion
    // hub. Angle is relative to where the robot started (0 deg).
    public static void turnToAngle(double angle, double v, double threshold) throws InterruptedException {

        // While the robot's heading is outside the threshold of the desired angle (+- the threshold),
        // turn in the appropriate direction at a slow turn speed
        while ((mRobot.getHeading() > (angle + threshold) || mRobot.getHeading() < (angle - threshold))
                && RobotHardware.kActiveAuto.opModeIsActive()) {

            RobotHardware.kActiveAuto.telemetry.addData("Heading", mRobot.getHeading());
            RobotHardware.kActiveAuto.telemetry.update();

            if (mRobot.getHeading() < angle) {
                mRobot.mecanumDrive(RobotHardware.DriveMode.TURN_LEFT,
                        v, 0);
            } else {
                mRobot.mecanumDrive(RobotHardware.DriveMode.TURN_RIGHT,
                        v, 0);
            }
        }

        // Stop the drive motors and stop turning once the loop has exited
        mRobot.stopDrive();
    }

    // Action for operating the mecanumDrive() low-level method for a certain amount of
    // time. The robot will use the IMU to keep its heading.
    public static void driveByTime(RobotHardware.DriveMode mode, double v, long time) throws InterruptedException {

        // Store the system time and original heading at the start of the action
        long startTime = System.currentTimeMillis();
        double desiredHeading = mRobot.getHeading();

        // Only keep the action running while the elapsed time is less than the desired time
        // to drive
        while (System.currentTimeMillis() - startTime < time && RobotHardware.kActiveAuto.opModeIsActive()) {

            // Calculate a proportional "turn" value to adjust any heading error that occurs
            // while driving
            double gyroHeading = mRobot.getHeading();
            double angleDifference = boundHalfDegrees
                    (desiredHeading - gyroHeading);
            double turn = 0.8 * (-1.0/80.0) * angleDifference;

            // Drive the motors with this turn offset
            mRobot.mecanumDrive(mode, v, turn);
        }

        // Stop driving once time has expired
        mRobot.stopDrive();
    }

    // Action for driving to a desired encoder position. The robot will use the front
    // left motor for determining position/when to stop, with all the motors running
    // in RUN_WITH_ENCODER to keep their velocities identical. Heading is also corrected
    // with the IMU
    public static void driveToPosition(RobotHardware.DriveMode mode, double v, double target) throws InterruptedException {

        // Store the starting heading of robot
        double desiredHeading = mRobot.getHeading();
        double direction;

        // Driving forward, strafing right, and turning right all spin the FL motor forward,
        // so its encoder reading goes up as we proceed, which is what we want. However, the
        // other 3 drive modes spin the FL motors backwards as they proceed, so I set our
        // direction multiplier to -1.
        switch (mode) {
            case FORWARD:
            case STRAFE_RIGHT:
            case TURN_RIGHT:
            default:
                direction = (mRobot.getDriveCounts() < target) ? 1.0 : -1.0;
                break;
            case BACKWARD:
            case STRAFE_LEFT:
            case TURN_LEFT:
                direction = (mRobot.getDriveCounts() < target) ? -1.0 : 1.0;
                break;
        }

        // Reset encoder counts before running
        mRobot.resetEncoders();

        // Run this action until the FL encoder reaches the target position
        if (mode != RobotHardware.DriveMode.BACKWARD) {
            while (mRobot.getDriveCounts() * direction < target && RobotHardware.kActiveAuto.opModeIsActive()) {

                // Driving forward, strafing right, and turning right all spin the FL motor forward,
                // so its encoder reading goes up as we proceed, which is what we want. However, the
                // other 3 drive modes spin the FL motors backwards as they proceed, so I set our
                // direction multiplier to -1.
                switch (mode) {
                    case FORWARD:
                    case STRAFE_RIGHT:
                    case TURN_RIGHT:
                    default:
                        direction = (mRobot.getDriveCounts() < target) ? 1.0 : -1.0;
                        break;
                    case BACKWARD:
                    case STRAFE_LEFT:
                    case TURN_LEFT:
                        direction = (mRobot.getDriveCounts() < target) ? -1.0 : 1.0;
                        break;
                }

                // Calculate the proportional heading error and correct for it while driving
                double gyroHeading = mRobot.getHeading();
                double angleDifference = boundHalfDegrees
                        (desiredHeading - gyroHeading);
                double turn = 0.8 * (-1.0 / 80.0) * angleDifference;

                mRobot.mecanumDrive(mode, direction * v, 0);
            }
        } else {
            while (mRobot.getDriveCounts() > target && RobotHardware.kActiveAuto.opModeIsActive()) {
                mRobot.mecanumDrive(RobotHardware.DriveMode.BACKWARD, v, 0);
            }
        }

        // Stop driving once the target position has been reached
        mRobot.stopDrive();
    }

    public static void setRampPosition(double position) {
        boolean pastTarget = false;
        mRobot.resetRampEncoder();
        while (!pastTarget && RobotHardware.kActiveAuto.opModeIsActive()) {
            if (position < 0) {
                mRobot.setRamp(-Constants.RAMP_SPEED);
                pastTarget = mRobot.getRampEncoder() < position;
            } else {
                mRobot.setRamp(Constants.RAMP_SPEED);
                pastTarget = mRobot.getRampEncoder() > position;
            }
        }

        mRobot.setRamp(0);

    }


    // Binds an angle to -180 to 180 deg to avoid drastic turns while correcting heading
    public static double boundHalfDegrees(double angle_degrees) {
        while (angle_degrees >= 180.0) angle_degrees -= 360.0;
        while (angle_degrees < -180.0) angle_degrees += 360.0;

        return angle_degrees;
    }
}


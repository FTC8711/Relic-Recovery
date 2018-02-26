package org.firstinspires.ftc.teamcode.Auto;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Created by 21maffetone on 2/25/18.
 */

public class Commands {

    private static RobotHardware mRobot;

    public static void init(RobotHardware robot) {
        mRobot = robot;
    }

    public static void turnToAngle(double angle) throws InterruptedException {
        while (mRobot.getHeading() > (angle + 2.0) && mRobot.getHeading() < (angle - 2.0)) {
            if (mRobot.getHeading() < angle) {
                mRobot.mecanumDrive(RobotHardware.DriveMode.TURN_RIGHT,
                        Constants.DRIVE_TURN_SPEED, 0);
            } else {
                mRobot.mecanumDrive(RobotHardware.DriveMode.TURN_LEFT,
                        Constants.DRIVE_TURN_SPEED, 0);
            }

            mRobot.stopDrive();
        }
    }

    public static void driveByTime(RobotHardware.DriveMode mode, double v, long time)
            throws InterruptedException {
        long startTime = System.currentTimeMillis();
        double desiredHeading = mRobot.getHeading();

        while (System.currentTimeMillis() - startTime > time) {

            double gyroHeading = mRobot.getHeading();
            double angleDifference = boundHalfDegrees
                    (desiredHeading - gyroHeading);
            double turn = 0.8 * (-1.0/80.0) * angleDifference;

            mRobot.mecanumDrive(mode, v, turn);
        }
    }

    public static void driveToPosition(RobotHardware.DriveMode mode, double v,
                                       double target) throws InterruptedException {
        while (mRobot.getDriveCounts() != target) {
            double desiredHeading = mRobot.getHeading();
            double direction;

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

            double gyroHeading = mRobot.getHeading();
            double angleDifference = boundHalfDegrees
                    (desiredHeading - gyroHeading);
            double turn = 0.8 * (-1.0/80.0) * angleDifference;

            mRobot.mecanumDrive(mode, direction * v, turn);
        }
    }

    private static double boundHalfDegrees(double angle_degrees) {
        while (angle_degrees >= 180.0) angle_degrees -= 360.0;
        while (angle_degrees < -180.0) angle_degrees += 360.0;
        return angle_degrees;
    }
}


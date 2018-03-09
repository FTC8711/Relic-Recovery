package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Created by 21maffetone on 2/25/18.
 *
 * Operational Mode for the autonomous period of the game, where we are on the RED alliance
 * and are on the "close" balancing stone RELATIVE TO THE RELIC RECOVERY ZONE/AUDIENCE.
 */

@Autonomous(name = "Red - Close", group = "Competition Auto")

public class RedCloseAutoMode extends LinearOpMode {

    private RelicRecoveryVuMark cryptokey;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware();
        RobotHardware.kActiveAuto = this;

        // Set the start position across all instances of RobotHardware to determine how
        // the robot will align to the cryptobox
        RobotHardware.kStartPosition = RobotHardware.StartPosition.RED_CLOSE;

        // Initialize the robot's hardware and the autonomous "actions"
        robot.init(hardwareMap);
        Actions.init(robot);

        // Start up Vuforia tracking before starting the match to allow it time to start up
        robot.activateTracking();

        // Holds the OpMode here until play is pressed by the driver
        waitForStart();
        telemetry.addData("Status", "Initialized");

        // Start up the internal IMU to be able to get readings
        robot.startAccelerationIntegration();

        timer.reset();

        cryptokey = RelicRecoveryVuMark.UNKNOWN;
        // Search for the cryptokey until it is found, timing out after 2 seconds
        while ((timer.seconds() < 2 && cryptokey == RelicRecoveryVuMark.UNKNOWN)
                && opModeIsActive()) {
            cryptokey = robot.getCryptokey();
            telemetry.addData("Cryptokey: ", cryptokey);
        }

        // Lower the jewel arm and wait 1.5 seconds before proceeding to allow it to lower
        robot.setJewelArm(Constants.JEWEL_ARM_READ);
        sleep(1500);

        // If the jewel behind the robot is red, simply drive forward a bit to knock it off,
        // then drive all the way off the balancing stone
        if (robot.getJewelColorHue() > Constants.RED_MIN_THRESHOLD) {
            telemetry.addData("Path", "Jewel is RED");

            Actions.driveToPosition(RobotHardware.DriveMode.FORWARD, 0.3, 600);
            robot.setJewelArm(Constants.JEWEL_ARM_STOW);
            Actions.driveToPosition(RobotHardware.DriveMode.FORWARD, 0.3, 200);

            // If its blue, turn the other direction to knock it off and then drive off the
            // balancing stone
        } else if (robot.getJewelColorHue() > Constants.BLUE_MIN_THRESHOLD) {
            telemetry.addData("Path", "Jewel is BLUE");
            Actions.turnToAngle(-5.0, 0.3, 2);

            sleep(500);
            // Turn back to the starting angle (0 deg) and raise the jewel arm back up
            robot.setJewelArm(Constants.JEWEL_ARM_STOW);
            Actions.turnToAngle(0, 0.3, 2);

            Actions.driveToPosition(RobotHardware.DriveMode.FORWARD, 0.3, 800);

            // Finally, if we get no reading assume that the jewel is red since the sensor
            // usually is just missing a red jewel if it gets no reading
        } else {
            telemetry.addData("Path", "Jewel is RED (no reading)");
            Actions.driveToPosition(RobotHardware.DriveMode.FORWARD, 0.3, 600);
            robot.setJewelArm(Constants.JEWEL_ARM_STOW);
            Actions.driveToPosition(RobotHardware.DriveMode.FORWARD, 0.3, 200);

        }

        sleep(500);
        // Correct angle again before proceeded
        Actions.turnToAngle(0, 0.3, 1.5);

        // Drive forward to about the first cryptobox column
        Actions.driveToPosition(RobotHardware.DriveMode.FORWARD, 0.3, 500);
        // Turn so that the back of the bot is facing the cryptobox
        Actions.turnToAngle(90, 0.5, 2);


        // TODO: Implement glyph scoring portion of auto

        double wallDistance = robot.getRangeDistance();
        double desiredHeading = robot.getHeading();

        int columnsPassed = 0;
        int columnTarget = 1;

        switch (cryptokey) {
            case LEFT:
                columnTarget = 3;
            case CENTER:
                columnTarget = 2;
            case RIGHT:
            default:
                columnTarget = 1;
        }

        while (columnsPassed < columnTarget && opModeIsActive()) {
            telemetry.addData("Distance", robot.getRangeDistance());
            telemetry.addData("Columns", columnsPassed);
            telemetry.update();
            // Calculate a proportional "turn" value to adjust any heading error that occurs
            // while driving
            double gyroHeading = robot.getHeading();
            double angleDifference = Actions.boundHalfDegrees
                    (desiredHeading - gyroHeading);
            double turn = 0.7 * (-1.0/80.0) * angleDifference;

            // Drive the motors with this turn offset
            robot.mecanumDrive(RobotHardware.DriveMode.STRAFE_RIGHT, 0.55, turn);

            if (robot.getRangeDistance() <= wallDistance - 6) {
                columnsPassed++;
                sleep(200);
            }
        }

        Actions.turnToAngle(90, 0.3, 1);
        Actions.turnToAngle(90, 0.3, 1);

        sleep(500);

        Actions.driveToPosition(RobotHardware.DriveMode.FORWARD, 0.4, 200);

        robot.setIntake(Constants.INTAKE_SPEED);
        robot.setRamp(-0.4);
        sleep(700);
        robot.setRamp(0);
        robot.setIntake(0);

        sleep(200);

        robot.setRamp(Constants.RAMP_SPEED);
        sleep(2300);
        robot.setRamp(0);

        Actions.driveByTime(RobotHardware.DriveMode.BACKWARD, 0.4, 800);
        Actions.driveByTime(RobotHardware.DriveMode.FORWARD, 0.4, 800);

//        Actions.driveToPosition(RobotHardware.DriveMode.FORWARD, -0.4, -400);

//        robot.setIntake(Constants.INTAKE_SPEED);
//        robot.setRamp(-Constants.RAMP_SPEED);
//        sleep(1000);
//        robot.setRamp(0);
//        robot.setIntake(0);

//        int columnsCounted = 0;
//        int targetColumns = 0;
//
//        switch (cryptokey) {
//            case CENTER:
//                targetColumns = 1;
//                break;
//            case RIGHT:
//                targetColumns = 2;
//                break;
//        }
//
//        // If the column is RIGHT (closest to balancing stone), we simply need to strafe
//        // left until we hit a column (sensor is on left side of bot)
//        if (cryptokey == RelicRecoveryVuMark.RIGHT) {
//            while (robot.getRangeDistance() >
//                    Constants.CRYPTOKEY_COLUMN_MAX_DIST_THRESHOLD && opModeIsActive()) {
//                robot.mecanumDrive(RobotHardware.DriveMode.STRAFE_LEFT, 0.3, 0);
//            }
//            // Otherwise, we need to strafe left. The sensor should pass one
//            // column to get to center, and two to get to left
//        } else {
//            while (columnsCounted < targetColumns && opModeIsActive()) {
//                robot.mecanumDrive(RobotHardware.DriveMode.STRAFE_RIGHT, 0.3, 0);
//
//                // If we hit a column, increase columns counted and wait 0.2 seconds to
//                // let the bot get past it before checking again
//                if (robot.getRangeDistance() > Constants.CRYPTOKEY_COLUMN_MAX_DIST_THRESHOLD) {
//                    columnsCounted++;
//                    sleep(200);
//                }
//            }
//        }

        robot.stopDrive();
    }
}

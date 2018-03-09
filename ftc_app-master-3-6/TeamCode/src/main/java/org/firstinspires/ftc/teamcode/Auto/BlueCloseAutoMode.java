package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Auto.Actions;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Created by 21maffetone on 2/25/18.
 *
 * Operational Mode for the autonomous period of the game, where we are on the RED alliance
 * and are on the "close" balancing stone RELATIVE TO THE RELIC RECOVERY ZONE/AUDIENCE.
 */

@Autonomous(name = "Blue - Close", group = "Competition Auto")

public class BlueCloseAutoMode extends LinearOpMode {

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

        timer.reset();

        // Start up the internal IMU to be able to get readings
        robot.startAccelerationIntegration();


        // Search for the cryptokey until it is found, timing out after 2 seconds
        while ((timer.seconds() < 2 && cryptokey == RelicRecoveryVuMark.UNKNOWN)
                && opModeIsActive()) {
            cryptokey = robot.getCryptokey();
            telemetry.addData("Cryptokey: ", cryptokey);
            telemetry.update();
        }

        // Lower the jewel arm and wait 1.5 seconds before proceeding to allow it to lower
        robot.setJewelArm(Constants.JEWEL_ARM_READ);
        sleep(1500);

        // If the jewel behind the robot is red, turn slightly right to knock the blue jewel
        // off
        if (robot.getJewelColorHue() > Constants.RED_MIN_THRESHOLD) {
            telemetry.addData("Path", "Jewel is RED");
            telemetry.update();

            Actions.driveToPosition(RobotHardware.DriveMode.BACKWARD, 0.3, 400);
            robot.setJewelArm(Constants.JEWEL_ARM_STOW);
            Actions.driveToPosition(RobotHardware.DriveMode.BACKWARD, 0.3, 200);

            // If its blue, turn the other direction to knock it off
        } else if (robot.getJewelColorHue() > Constants.BLUE_MIN_THRESHOLD) {
            telemetry.addData("Path", "Jewel is BLUE");
            telemetry.update();

            Actions.turnToAngle(7.0, 0.3, 2);

            sleep(500);
            // Turn back to the starting angle (0 deg) and raise the jewel arm back up
            robot.setJewelArm(Constants.JEWEL_ARM_STOW);
            Actions.turnToAngle(0, 0.3, 2);

            Actions.driveToPosition(RobotHardware.DriveMode.BACKWARD, 0.3, 600);

            // Finally, if we get no reading assume that the jewel is red since the sensor
            // usually is just missing a red jewel if it gets no reading
        } else {
            telemetry.addData("Path", "Jewel is RED (no reading)");
            telemetry.update();

            Actions.driveToPosition(RobotHardware.DriveMode.BACKWARD, 0.3, 400);
            robot.setJewelArm(Constants.JEWEL_ARM_STOW);
            Actions.driveToPosition(RobotHardware.DriveMode.BACKWARD, 0.3, 200);

        }



        sleep(500);
        Actions.turnToAngle(0, 0.2, 1);

        Actions.driveToPosition(RobotHardware.DriveMode.BACKWARD, 0.3, 500);
        Actions.turnToAngle(-90, 0.2, 1);

        // TODO: Implement glyph scoring portion of auto
//        Actions.driveToPosition(RobotHardware.DriveMode.FORWARD,
//                Constants.DRIVE_NORMAL_SPEED, Paths.CLOSE_STONE_TO_FIELD_FLOOR);
//
//        Actions.turnToAngle(90);
    }
}

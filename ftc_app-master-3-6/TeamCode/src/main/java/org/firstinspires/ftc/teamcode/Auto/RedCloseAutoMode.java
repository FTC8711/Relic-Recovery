package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Created by 21maffetone on 2/25/18.
 */

@Autonomous(name = "Red - Close", group = "Competition Auto")

public class RedCloseAutoMode extends LinearOpMode {

    private RelicRecoveryVuMark cryptokey;
    private ElapsedTime timer = new ElapsedTime();

    private double startHeading;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware();

        robot.init(hardwareMap);
        Commands.init(robot);

        robot.activateTracking();

        waitForStart();
        telemetry.addData("Status", "Initialized");

        timer.reset();

        robot.startAccelerationIntegration();

        while (timer.seconds() < 2 && opModeIsActive()) {
            cryptokey = robot.getCryptokey();
            telemetry.addData("Cryptokey: ", cryptokey);
        }

        robot.setJewelArm(Constants.JEWEL_ARM_READ);
        sleep(1500);

        if (robot.getJewelColorHue() > Constants.RED_MIN_THRESHOLD) {
            telemetry.addData("Path", "Jewel is RED");
            Commands.driveToPosition(RobotHardware.DriveMode.FORWARD,
                    Constants.DRIVE_NORMAL_SPEED, 1000);
        } else if (robot.getJewelColorHue() > Constants.BLUE_MIN_THRESHOLD) {
            telemetry.addData("Path", "Jewel is BLUE");
            Commands.turnToAngle(-25.0);
            Commands.turnToAngle(0);
        } else {
            telemetry.addData("Path", "Jewel is RED (no reading)");
            Commands.driveToPosition(RobotHardware.DriveMode.FORWARD,
                    Constants.DRIVE_NORMAL_SPEED, 1000);
        }

        robot.setJewelArm(Constants.JEWEL_ARM_STOW);

    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.InterruptedIOException;

/**
 * Created by 21maffetone on 12/28/17.
 */

@TeleOp(name = "TeleOp - Threadded", group = "Experimental TeleOp")
@Disabled

public class TeleOpThreadded extends LinearOpMode {
    MainHardware robot = new MainHardware();
    private ElapsedTime runtime  = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            Thread control = new Thread() {
                public void run() {
                    double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                    double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                    double rightX = gamepad1.right_stick_x;
                    final double v1 = r * Math.cos(robotAngle) + rightX;
                    final double v2 = r * Math.sin(robotAngle) - rightX;
                    final double v3 = r * Math.sin(robotAngle) + rightX;
                    final double v4 = r * Math.cos(robotAngle) - rightX;

                    robot.frontLeft.setPower(v1);
                    robot.frontRight.setPower(v2);
                    robot.backLeft.setPower(v3);
                    robot.backRight.setPower(v4);

                    if (gamepad1.right_bumper) {
                        robot.intakeL.setPower(1);
                        robot.intakeR.setPower(1);
                    } else {
                        robot.intakeL.setPower(0);
                        robot.intakeR.setPower(0);
                    }

                    if (gamepad1.dpad_up) {
                        robot.ramp.setPower(1);
                    } else {
                        robot.ramp.setPower(0);
                    }

                    if (gamepad1.dpad_down) {
                        robot.ramp.setPower(-1);
                    } else {
                        robot.ramp.setPower(0);
                    }
                }
            };

            Thread automatedScoring = new Thread() {
                public void run() {
                    if (gamepad1.x) {
                        robot.ramp.setPower(1);
                        safeSleep(1000);
                        robot.ramp.setPower(0);

                        robot.ramp.setPower(-1);
                        safeSleep(1000);
                        robot.ramp.setPower(0);
                    }
                }
            };

            control.start();
            automatedScoring.start();
        }
    }

    private void safeSleep(long m) {
        sleep(m); // unfortunate naming in a thread
    }

}

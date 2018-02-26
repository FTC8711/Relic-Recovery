package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by 21maffetone on 2/25/18.
 */

@TeleOp (name = "TeleOp - Main", group = "Compeition TeleOp")

public class TeleOpMain extends OpMode {
    private RobotHardware robot = new RobotHardware();
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        timer.startTime();
        telemetry.addData("Status", "Running");
    }

    @Override
    public void loop() {
        robot.setJewelArm(Constants.JEWEL_ARM_STOW);

        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;

        double rightX = gamepad1.right_stick_x;
        double v1 = r * Math.cos(robotAngle) + rightX;
        double v2 = r * Math.sin(robotAngle) - rightX;
        double v3 = r * Math.sin(robotAngle) + rightX;
        double v4 = r * Math.cos(robotAngle) - rightX;

        robot.setDrive(v1, v2, v3, v4);

        if (gamepad1.right_bumper) {
            robot.setIntake(Constants.INTAKE_SPEED);
        } else if (gamepad1.left_bumper){
            robot.setIntake(-Constants.INTAKE_SPEED);
        } else {
            robot.setIntake(0);
        }

        if (gamepad2.dpad_up) {
            robot.setRamp(-Constants.RAMP_SPEED);
        } else if (gamepad2.dpad_down){
            robot.setRamp(Constants.RAMP_SPEED);
        } else {
            robot.setRamp(0);
        }

        if (gamepad2.dpad_left) {
            robot.setRelicSlide(Constants.EXTENDER_SPEED);
        } else if (gamepad2.dpad_right) {
            robot.setRelicSlide(-Constants.EXTENDER_SPEED);
        } else {
            robot.setRelicSlide(0);
        }

        if (gamepad2.x) {
            robot.setRelicPivot(Constants.RELIC_PIVOT_UP);
        } else if (gamepad2.b) {
            robot.setRelicPivot(Constants.RELIC_PIVOT_DOWN);
        }


        if (gamepad2.a) {
            robot.setRelicGrabber(Constants.RELIC_GRABBER_CLOSED);
        } else if (gamepad2.y) {
            robot.setRelicGrabber(Constants.RELIC_GRABBER_OPEN);
        }

    }
}

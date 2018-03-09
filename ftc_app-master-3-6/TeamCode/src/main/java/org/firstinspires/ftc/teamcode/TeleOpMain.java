package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 21maffetone on 2/25/18.
 *
 * Main/Only Operational Mode for the teleoperated period of the game. Runs in an iterative
 * loop to allow for real-time control from the drivers and real-time data being sent back.
 * Program is SINGLE-THREADED for now, but will most likely move to multi-threaded upon
 * determining useful sequences to automate (e.g. aligning to cryptobox)
 */

@TeleOp (name = "TeleOp - Main", group = "Compeition TeleOp")

public class TeleOpMain extends OpMode {
    private RobotHardware robot = new RobotHardware();
    private ElapsedTime timer = new ElapsedTime();

    // Routine that runs ONCE upon the driver pressing "Init"
    @Override
    public void init() {
        // Initialize all the robot's hardware and get it ready to run
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    // Routine that runs ONCE after the driver presses "play"
    @Override
    public void start() {
        timer.startTime();
        telemetry.addData("Status", "Running");
    }

    // Routine the runs CONTINUOUSLY/PERIODICALLY during the OpMode
    @Override
    public void loop() {
        // Diagnostics, will not be included in final competition code
        telemetry.addData("Range Sensor Distance (cm)", robot.getRangeDistance());
        telemetry.addData("Heading", robot.getHeading());
        telemetry.addData("Encoder Position (FL)", robot.getDriveCounts());


        // Keep the jewel arm in the up position constantly during TeleOp to prevent it from
        // being damaged
        robot.setJewelArm(Constants.JEWEL_ARM_STOW);

        // Determine the desired angle of where the driver wants to go based on the LEFT
        // joystick
        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;

        // Calculate the motor values to achieve this desired angle, taking into account
        // the turn from the RIGHT joystick
        double v1 = r * Math.cos(robotAngle) + rightX;
        double v2 = r * Math.sin(robotAngle) - rightX;
        double v3 = r * Math.sin(robotAngle) + rightX;
        double v4 = r * Math.cos(robotAngle) - rightX;

        // Write the values to the drive motors
        robot.setDrive(v1, v2, v3, v4);

        // Driver 1's left/right bumpers run the intake in/out WHILE PRESSED
        if (gamepad1.right_bumper) {
            robot.setIntake(Constants.INTAKE_SPEED);
        } else if (gamepad1.left_bumper){
            robot.setIntake(-Constants.INTAKE_SPEED);
        } else {
            robot.setIntake(0);
        }

        // Driver 2's up/down on the dpad runs the ramp up/down WHILE PRESSED
        if (gamepad2.dpad_up) {
            robot.setRamp(-Constants.RAMP_SPEED);
        } else if (gamepad2.dpad_down){
            robot.setRamp(Constants.RAMP_SPEED);
        } else {
            robot.setRamp(0);
        }

        // Driver 2's left/right on the dpad runs the relic extender in/out WHILE PRESSED
        if (gamepad2.dpad_left) {
            robot.setRelicSlide(Constants.EXTENDER_SPEED);
        } else if (gamepad2.dpad_right) {
            robot.setRelicSlide(-Constants.EXTENDER_SPEED);
        } else {
            robot.setRelicSlide(0);
        }

        // Driver 2's x/b buttons rotates the entire relic possession mechanism to the
        // up/down position
        if (gamepad2.x) {
            robot.setRelicPivot(Constants.RELIC_PIVOT_UP);
        } else if (gamepad2.b) {
            robot.setRelicPivot(Constants.RELIC_PIVOT_DOWN);
        }

        // Driver 2's a/y buttons closes/opens the relic grabber to secure the relic
        if (gamepad2.a) {
            robot.setRelicGrabber(Constants.RELIC_GRABBER_CLOSED);
        } else if (gamepad2.y) {
            robot.setRelicGrabber(Constants.RELIC_GRABBER_OPEN);
        }

//        if (gamepad1.a) {
//            robot.setDrive(1, 0, 0, 0);
//        } else if (gamepad1.x) {
//            robot.setDrive(0, 1, 0, 0);
//        } else if (gamepad1.y) {
//            robot.setDrive(0, 0, 1, 0);
//        } else if (gamepad1.b) {
//            robot.setDrive(0, 0, 0, 1);
//        } else {
//            robot.stopDrive();
//        }
    }
}

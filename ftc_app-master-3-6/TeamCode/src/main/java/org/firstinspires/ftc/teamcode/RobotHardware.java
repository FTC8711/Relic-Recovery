package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by 21maffetone on 2/25/18.
 */

public class RobotHardware {

    public enum StartPosition {
        RED_CLOSE,
        RED_FAR,
        BLUE_CLOSE,
        BLUE_FAR
    }

    public static StartPosition kStartPosition;
    public static LinearOpMode kActiveAuto;

    // Drive subsystem
    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveBackLeft;
    private DcMotor driveBackRight;
    private BNO055IMU imu;

    public enum DriveMode {
        FORWARD,
        BACKWARD,
        STRAFE_LEFT,
        STRAFE_RIGHT,
        TURN_LEFT,
        TURN_RIGHT
    }

    // Intake subsystem
    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    // Glyph scoring subsystem
    private DcMotor rampActuator;
    private ModernRoboticsI2cRangeSensor rangeSensorBack;

    // Relic subsystem
    private DcMotor relicSlideSpool;
    private Servo relicGrabPivot;
    private Servo relicGrabGrabber;

    // Jewel subsystem
    private Servo jewelArmActuator;
    private ColorSensor jewelColorSensor;

    // Vuforia/vision subsystem
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;

    private HardwareMap mHwMap;

    public void init(HardwareMap hwMap) {
        mHwMap = hwMap;

        // Initialize drive subsystem motors from the hardware map
        driveFrontLeft = mHwMap.get(DcMotor.class, "D0");
        driveFrontRight = mHwMap.get(DcMotor.class, "D1");
        driveBackLeft = mHwMap.get(DcMotor.class, "D2");
        driveBackRight = mHwMap.get(DcMotor.class, "D3");

        // Reset drive subsystem encoders
        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set drive subsystem encoders to achieve velocity using encoders
        driveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set drive subsystem encoders to apply resistance to outside forces
        // upon receiving zero power
        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        driveBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Ensure all drive subsystem motors are at zero power/stopped
        driveFrontLeft.setPower(0);
        driveFrontRight.setPower(0);
        driveBackLeft.setPower(0);
        driveBackRight.setPower(0);

        // Initialize the imu from the hardware map with the parameters previously defined
        imu = mHwMap.get(BNO055IMU.class, "IMU");

        // Initialize intake subsystem motors from the hardware map
        intakeLeft = mHwMap.get(DcMotor.class, "I0");
        intakeRight = mHwMap.get(DcMotor.class, "I1");

        // Set intake subsystem motors to not use encoders as they are not connected
        intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set intake subsystem motors to float or "coast" upon receiving zero power, and
        // not attempt resistance
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Ensure all intake subsystem motors are at zero power/stopped
        intakeLeft.setPower(0);
        intakeRight.setPower(0);

        // Initialize glyph scoring subsystem motor from the hardware map
        rampActuator = mHwMap.get(DcMotor.class, "G0");

        rampActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set glyph scoring subsystem motor to not use encoders as they are not connected
        rampActuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set glyph scoring subsystem motor to apply resistance to outside forces upon
        // receiving zero power to prevent backtracking of the ramp
        rampActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // Ensure all glyph scoring subsystem motors are at zero power/stopped
        rampActuator.setPower(0);

        // Initialize the glyph scoring subsystem range sensor from the hardware map
        rangeSensorBack = mHwMap.get(ModernRoboticsI2cRangeSensor.class,
                "BACK_RANGE");

        // Initialize relic subsystem motor from the hardware map
        relicSlideSpool = mHwMap.get(DcMotor.class, "R0");

        // Set relic subsystem motor to not use encoders as they are not connected
        relicSlideSpool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set relic subsystem motor to apply resistance to outside forces upon
        // receiving zero power to prevent backtracking of the slide
        relicSlideSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ensure all relic subsystem motors are at zero power/stopped
        relicSlideSpool.setPower(0);

        // Initialize the relic subsystem servos from the hardware map
        relicGrabPivot = mHwMap.get(Servo.class, "RELIC_PIVOT");
        relicGrabGrabber = mHwMap.get(Servo.class, "RELIC_GRABBER");

        // Set the direction of the relic subsystem servos respectively
        relicGrabPivot.setDirection(Servo.Direction.FORWARD);
        relicGrabGrabber.setDirection(Servo.Direction.FORWARD);

        // Set the respective starting position of each relic subsystem servo
        relicGrabPivot.setPosition(Constants.RELIC_PIVOT_DOWN);
        relicGrabGrabber.setPosition(Constants.RELIC_GRABBER_OPEN);

        // Initialize the jewel subsystem servo from the hardware map
        jewelArmActuator = mHwMap.get(Servo.class, "JEWEL_ACTUATOR");

        // Set the direction of the jewel subsystem servo
        jewelArmActuator.setDirection(Servo.Direction.FORWARD);

        // Set the respective starting position of the jewel subsystem servo
        jewelArmActuator.setPosition(Constants.JEWEL_ARM_STOW);

        // Initialize the jewel subsystem color sensor
        jewelColorSensor = hwMap.get(ColorSensor.class, "JEWEL_COLOR");

        // Enable the led of the jewel subsystem color sensor by default to aid in reading
        jewelColorSensor.enableLed(true);
    }

    public void vuforiaInit() {
        int cameraMonitorViewId = mHwMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id",
                        mHwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vufoiraParameters = new VuforiaLocalizer.
                Parameters(cameraMonitorViewId);

        vufoiraParameters.vuforiaLicenseKey = Constants.VUFORIA_KEY;
        vufoiraParameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(vufoiraParameters);

        relicTrackables = this.vuforia.
                loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
    }

    public void setDrive(double v1, double v2, double v3, double v4) {
        driveFrontLeft.setPower(v1);
        driveFrontRight.setPower(v2);
        driveBackLeft.setPower(v3);
        driveBackRight.setPower(v4);
    }

    public void stopDrive() {
        setDrive(0, 0, 0, 0);
    }

    public void mecanumDrive(DriveMode direction, double v, double offset) {
        switch (direction) {
            case FORWARD:
                setDrive(v - offset, v + offset, v - offset, v + offset);
                break;
            case BACKWARD:
                setDrive(-v - offset, -v + offset, -v - offset, -v + offset);
                break;
            case STRAFE_LEFT:
                setDrive(-v - offset, v + offset, v - offset, -v + offset);
                break;
            case STRAFE_RIGHT:
                setDrive(v - offset, -v + offset, -v - offset, v + offset);
                break;
            case TURN_LEFT:
                setDrive(-v - offset, v + offset, -v - offset, v + offset);
                break;
            case TURN_RIGHT:
                setDrive(v - offset, -v + offset, v - offset, -v + offset);
                break;
        }
    }

    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public void startAccelerationIntegration() {
        // Define parameters for the internal IMU
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = true;
        imuParameters.loggingTag = "IMU";
        imuParameters.accelerationIntegrationAlgorithm
                = new JustLoggingAccelerationIntegrator();

        imu.initialize(imuParameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public void setIntake(double v) {
        intakeLeft.setPower(v);
        intakeRight.setPower(v);
    }

    public void setRamp(double v) {
        rampActuator.setPower(v);
    }

    public double getRampEncoder() {
        return rampActuator.getCurrentPosition();
    }

    public void resetRampEncoder() {
        rampActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rampActuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getRangeDistance() {
        return rangeSensorBack.getDistance(DistanceUnit.CM);
    }

    public void setRelicSlide(double v) {
        relicSlideSpool.setPower(v);
    }

    public void setRelicPivot(double p) {
        relicGrabPivot.setPosition(p);
    }

    public void setRelicGrabber(double p) {
        relicGrabGrabber.setPosition(p);
    }

    public void setJewelArm(double p) {
        jewelArmActuator.setPosition(p);
    }

    public float getJewelColorHue() {
        float hsvValues[] = {0F,0F,0F};
        Color.RGBToHSV(jewelColorSensor.red(), jewelColorSensor.green(),
                jewelColorSensor.blue(), hsvValues);

        return hsvValues[0];
    }

    public int getDriveCounts() {
        return driveFrontLeft.getCurrentPosition();
    }

    public void resetEncoders() {
        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void activateTracking() {
        relicTrackables.activate();
    }
    public void deactivateTracking() {
        relicTrackables.deactivate();
    }

    public RelicRecoveryVuMark getCryptokey() {
        return RelicRecoveryVuMark.from(relicTemplate);
    }
}

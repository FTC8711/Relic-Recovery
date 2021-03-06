/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.components.Pid;

import java.util.Locale;

public class MainHardware
{
    /* Public OpMode members. */
    public DcMotor     frontLeft;
    public DcMotor     frontRight;
    public DcMotor     backRight;
    public DcMotor     backLeft;

    public DcMotor     intakeL;
    public DcMotor     intakeR;

    public DcMotor     relicExtender;
    public DcMotor     ramp;

    public Servo       jewelDiverter;
    public ColorSensor jewelColor;

    public Servo relicPivot;
    public Servo       relicGrab;

    public DistanceSensor wallDistance;

    public BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;

    public static final double INTAKE_SPEED = 1.0,
                               RAMP_SPEED = 0.45,
                               EXTENDER_SPEED = 1.0;

    public static final double JEWEL_START = 0.8;
    public static final double JEWEL_READ = 0.25;
    public static final double RED_THRESHOLD = 40;

    public static final double RELIC_GRAB_START = 0.7;
    public static final double RELIC_GRAB_GRABBED = 0;

    public static final double RELIC_PIVOT_START = 0;
    public static final double RELIC_PIVOT_UP = 1;

    public static final double DISTANCE_FROM_WALL = 4;
    public static final double DISTANCE_FROM_COLUMN = 2;

    public static  final double drivePidKp = 1;     // Tuning variable for PID.
    public static final double drivePidTi = 1.0;   // Eliminate integral error in 1 sec.
    public static final double  drivePidTd = 0.1;   // Account for error in 0.1 sec.

    // Protect against integral windup by limiting integral term.

    public static final double drivePidIntMax = 1.0;  // Limit to max speed.
    public static final double driveOutMax = 1.0;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public MainHardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeft = hwMap.get(DcMotor.class, "1");
        frontRight = hwMap.get(DcMotor.class, "2");
        backLeft = hwMap.get(DcMotor.class, "3");
        backRight = hwMap.get(DcMotor.class, "4");

        intakeL = hwMap.get(DcMotor.class, "intakeL");
        intakeR = hwMap.get(DcMotor.class, "intakeR");

        ramp = hwMap.get(DcMotor.class, "ramp");

        relicExtender = hwMap.get(DcMotor.class, "relic_extender");

        // Set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        intakeL.setPower(0);
        intakeR.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeL.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Define and initialize ALL installed servos.
        jewelDiverter = hwMap.get(Servo.class, "jewel");
        jewelDiverter.setPosition(JEWEL_START);

        relicPivot = hwMap.get(Servo.class, "relic_pivot");
        relicPivot.setPosition(RELIC_PIVOT_START);

        relicGrab = hwMap.get(Servo.class, "relic_grab");
        relicGrab.setPosition(RELIC_GRAB_START);

        jewelColor = hwMap.colorSensor.get("jewel_color");
        wallDistance = hwMap.get(DistanceSensor.class, "distance");

        // Define all parameters for and initialize the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


    }

    void manualDrive(double v1, double v2, double v3, double v4) {
        frontLeft.setPower(v1);
        frontRight.setPower(v2);
        backRight.setPower(v3);
        backLeft.setPower(v4);
    }

    void drive(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
    }

    void turnRight(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
        backLeft.setPower(power);
    }

    void turnLeft(double power) {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(-power);
    }

    void strafeLeft(double power) {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backRight.setPower(-power);
        backLeft.setPower(power);
    }

    void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    void runIntake(double power) {
        intakeL.setPower(power);
        intakeR.setPower(power);
    }

    void driveToDistance(double distance, double speed) {
        while (wallDistance.getDistance(DistanceUnit.INCH) < distance) {
            drive(speed);
        }

        stopDrive();
    }

    public static double boundHalfDegrees(double angle_degrees) {
        while (angle_degrees >= 180.0) angle_degrees -= 360.0;
        while (angle_degrees < -180.0) angle_degrees += 360.0;
        return angle_degrees;
    }



    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

   void composeTelemetry(Telemetry telemetry) {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
               });
//                .addData("roll", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.secondAngle);
//                    }
//                })
//                .addData("pitch", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.thirdAngle);
//                    }
//                });
//
//        telemetry.addLine()
//                .addData("grvty", new Func<String>() {
//                    @Override public String value() {
//                        return gravity.toString();
//                    }
//                })
//                .addData("mag", new Func<String>() {
//                    @Override public String value() {
//                        return String.format(Locale.getDefault(), "%.3f",
//                                Math.sqrt(gravity.xAccel*gravity.xAccel
//                                        + gravity.yAccel*gravity.yAccel
//                                        + gravity.zAccel*gravity.zAccel));
//                    }
//                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


 }


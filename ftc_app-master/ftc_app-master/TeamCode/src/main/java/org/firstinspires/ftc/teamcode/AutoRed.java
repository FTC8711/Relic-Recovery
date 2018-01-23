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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.MainHardware.JEWEL_READ;
import static org.firstinspires.ftc.teamcode.MainHardware.JEWEL_START;

@Autonomous(name="Red01", group="Competition Auto")

public class AutoRed extends LinearOpMode {

    // Declare OpMode members.
    MainHardware        robot   = new MainHardware();
    private ElapsedTime runtime = new ElapsedTime();
    private float hsvValues[] = {0F,0F,0F};

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    private enum Path {
        LEFT,
        CENTER,
        RIGHT
    }

    private Path path;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.jewelColor.enableLed(true);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "Ab7NcEP/////AAAAGU3lIT+010Tnsk66FEobCD4SlREK/jF55GNUnn41TQe4m7uCppwOboHMqXOsw13evfeXn/7ptt6Xk/Tl/hOpJDb+rEdawgaet7oln379ujuX4IpmkzjhcU6eaTtKVb6dYQYCK5nkSpKZU6o+pwii/qhfOQdekT1VArWa1WSrw7oXI2AM3KYXn8mSB+KHcsoeFMrFqqv5qDShrG81X3XbgxQFCbxIDsYsGnmRN5w5xoXBMm+bo8HjAlsmWWGZcEP294YBusc+X0645MPioUJalu/sGGJly4byQP7+bMcFyADhUEZz3UaYu/PCBVz6grWRd/OncikVkCFOojGf2fZq4riOQH7YaDLYmYee5Zs2a4jd"; //secure later (hopefully no one steals our key)

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

//        robot.composeTelemetry(telemetry);

        waitForStart();
        runtime.reset();

        relicTrackables.activate();

        while(runtime.seconds() < 2 && opModeIsActive()) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("VuMark", "%s visible", vuMark);

            switch (vuMark) {
                case LEFT:
                    telemetry.addData("Path", "Left");
                    path = Path.LEFT;
                    break;
                case CENTER:
                    telemetry.addData("Path", "Center");
                    path = Path.CENTER;
                    break;
                case RIGHT:
                    telemetry.addData("Path", "Right");
                    path = Path.RIGHT;
                    break;
                default:
                    path = Path.LEFT;
                    break;
            }

            telemetry.update();
        }


//        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        robot.jewelDiverter.setPosition(JEWEL_READ);

        sleep(1500);

        Color.RGBToHSV(robot.jewelColor.red(), robot.jewelColor.green(), robot.jewelColor.blue(), hsvValues);

        if (hsvValues[0] > 210) {
            telemetry.addData("Path ", "Jewel is RED -> Drive FORWARD");
            telemetry.addData("Hue ", hsvValues[0]);

            robot.drive(0.6);
            sleep(400);
            robot.stopDrive();

        }

        else if (hsvValues[0] > 60) {
            telemetry.addData("Path ", "Jewel is BLUE -> Drive BACKWARD");
            telemetry.addData("Hue ", hsvValues[0]);

            robot.turnRight(0.6);
            sleep(150);
            robot.stopDrive();

            robot.jewelDiverter.setPosition(JEWEL_START);

            robot.turnLeft(0.6);
            sleep(150);
            robot.stopDrive();

            robot.drive(0.6);
            sleep(400);
            robot.stopDrive();
        }

        else {
            telemetry.addData("Path ", "Jewel is RED -> Drive FORWARD");
            telemetry.addData("Hue ", hsvValues[0]);

            robot.drive(0.6);
            sleep(400);
            robot.stopDrive();
        }

        robot.jewelDiverter.setPosition(JEWEL_START);
        robot.jewelColor.enableLed(false);

        robot.drive(0.6);
        sleep(300);
        robot.stopDrive();

        robot.turnLeft(0.6);
        sleep(1300);
        robot.stopDrive();

        switch (path) {
            case LEFT:
                robot.strafeLeft(-0.6);
                sleep(400);
                robot.stopDrive();
                break;
            case CENTER:
                robot.strafeLeft(-0.6);
                sleep(700);
                robot.stopDrive();
                break;
            case RIGHT:
                robot.strafeLeft(-0.6);
                sleep(900);
                robot.stopDrive();
                break;
        }

        robot.intakeL.setPower(1);
        robot.intakeR.setPower(1);
        robot.ramp.setPower(-0.45);
        sleep(800);
        robot.ramp.setPower(0.45);
        robot.intakeL.setPower(0);
        robot.intakeR.setPower(0);
        sleep(800);
        robot.ramp.setPower(0);

        sleep(1200);

        robot.drive(-0.6);
        sleep(300);
        robot.stopDrive();

        robot.drive(0.6);
        sleep(200);
        robot.stopDrive();

        robot.stopDrive();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.update();
        }
    }
}

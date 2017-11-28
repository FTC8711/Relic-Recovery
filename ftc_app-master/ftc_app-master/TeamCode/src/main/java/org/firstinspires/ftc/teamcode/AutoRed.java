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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.MainHardware.JEWEL_READ;
import static org.firstinspires.ftc.teamcode.MainHardware.JEWEL_START;
import static org.firstinspires.ftc.teamcode.MainHardware.RED_THRESHOLD;

@Autonomous(name="Red01", group="Competition Auto")

public class AutoRed extends LinearOpMode {

    // Declare OpMode members.
    MainHardware        robot   = new MainHardware()
    private ElapsedTime runtime = new ElapsedTime()
    float hsvValues[] = {0F,0F,0F}


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.jewelColor.enableLed(true)

        telemetry.addData("Status", "Initialized")
        telemetry.update()

//        robot.composeTelemetry(telemetry);

        waitForStart();
        runtime.reset();

        mecanumDrive(1, 1, 1, 1, 2000)

//        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        /* robot.jewelDiverter.setPosition(JEWEL_READ);

        sleep(1500);

        Color.RGBToHSV(robot.jewelColor.red(), robot.jewelColor.green(), robot.jewelColor.blue(), hsvValues);

        if(hsvValues[0] > RED_THRESHOLD) {
            telemetry.addData("Path", "Jewel is BLUE -> Drive Backwards");
        }

        else {
            telemetry.addData("Path", "Jewel is RED -> Drive Forward");
        }

        robot.jewelDiverter.setPosition(JEWEL_START);
        robot.jewelColor.enableLed(false);

        */





        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.update();
        }
    }

    public void mecanumDrive(double v1, double v2, double v3, double v4, long time) {
        robot.frontLeft.setPower(v1);
        robot.frontRight.setPower(v2);
        robot.backRight.setPower(v3);
        robot.backLeft.setPower(v4);

        sleep(time);
//looks good my dude
    }




}

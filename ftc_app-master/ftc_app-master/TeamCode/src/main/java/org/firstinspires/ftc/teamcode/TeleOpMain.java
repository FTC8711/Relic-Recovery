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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.MainHardware.JEWEL_START;
import static org.firstinspires.ftc.teamcode.MainHardware.RELIC_GRAB_GRABBED;
import static org.firstinspires.ftc.teamcode.MainHardware.RELIC_GRAB_START;

@TeleOp(name="TeleOp - Main", group="Competition TeleOp")

public class TeleOpMain extends OpMode {

    MainHardware robot = new MainHardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        telemetry.addData("Status", "Running");
        runtime.reset();
    }

    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        robot.jewelDiverter.setPosition(JEWEL_START);

        robot.frontLeft.setPower(v1);
        robot.frontRight.setPower(v2);
        robot.backLeft.setPower(v3);
        robot.backRight.setPower(v4);

        telemetry.addData("r: ", r);
        telemetry.addData("robotAngle: ", robotAngle);
        telemetry.addData("rightX: ", rightX);

        if (gamepad1.right_bumper) {
            robot.intakeL.setPower(1);
            robot.intakeR.setPower(1);
        } else if (gamepad1.left_bumper){
            robot.intakeL.setPower(-1);
            robot.intakeR.setPower(-1);
        } else {
            robot.intakeL.setPower(0);
            robot.intakeR.setPower(0);
        }

        if (gamepad2.dpad_up) {
            robot.ramp.setPower(-0.45);
        } else if (gamepad2.dpad_down){
            robot.ramp.setPower(0.45);
        } else {
            robot.ramp.setPower(0);
        }

        if (gamepad2.dpad_left) {
            robot.relicExtender.setPower(1);
        } else if (gamepad2.dpad_right) {
            robot.relicExtender.setPower(-1);
        } else {
            robot.relicExtender.setPower(0);
        }

        if (gamepad2.x) {
            robot.relicPivot.setPower(1);
        } else if (gamepad2.b) {
            robot.relicPivot.setPower(-1);
        } else {
            robot.relicPivot.setPower(0);
        }

        if (gamepad2.a) {
            robot.relicGrab.setPosition(RELIC_GRAB_GRABBED);
        } else if (gamepad2.y) {
            robot.relicGrab.setPosition(RELIC_GRAB_START);
        }

        /*
         * DEBUGGING
         */

        if (gamepad1.y) {
            robot.frontRight.setPower(1);
        }

        if (gamepad1.x) {
            robot.frontLeft.setPower(1);
        }

        if (gamepad1.b) {
            robot.backRight.setPower(1);
        }

        if (gamepad1.a) {
            robot.backLeft.setPower(1);
        }

    }

    @Override
    public void stop() {

    }

}

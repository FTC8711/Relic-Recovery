package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.components.Pid;

import static org.firstinspires.ftc.teamcode.MainHardware.boundHalfDegrees;
import static org.firstinspires.ftc.teamcode.MainHardware.driveOutMax;
import static org.firstinspires.ftc.teamcode.MainHardware.drivePidIntMax;
import static org.firstinspires.ftc.teamcode.MainHardware.drivePidKp;
import static org.firstinspires.ftc.teamcode.MainHardware.drivePidTd;
import static org.firstinspires.ftc.teamcode.MainHardware.drivePidTi;

/**
 * Created by 21maffetone on 2/21/18.
 */

public class DriveToColumn extends LinearOpMode {

    MainHardware robot = new MainHardware();
    private ElapsedTime runtime = new ElapsedTime();

    private Pid leftDrive;
    private Pid rightDrive;

    private boolean searchingForColumn = true;
    private double driveSpeed = 0.7;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double prevTime = 0;

        waitForStart();

        double desiredHeading = robot.imu.getAngularOrientation().firstAngle;
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(),
                1000);


        // drive forwards in a straight line
        while (searchingForColumn && runtime.seconds() < 2 && opModeIsActive()) {

            double gyroHeading = robot.imu.getAngularOrientation().firstAngle;
            double angleDifference = boundHalfDegrees
                    (desiredHeading - gyroHeading);
            double turn = 0.8 * (-1.0/80.0) * angleDifference;

            robot.manualDrive(driveSpeed - turn,
                    driveSpeed + turn,
                    driveSpeed + turn,
                    driveSpeed - turn);
            }


    }

}

package org.firstinspires.ftc.teamCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group = "Bot")
//@Disabled
public class Autonomous extends LinearOpMode {

    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        robot.encoderDrive(10, 0.5);
        robot.gyroHold(0.5,90,3);
        robot.encoderDrive(10, 0.5);
        robot.gyroHold(0.5,180,3);
        robot.encoderDrive(10, 0.5);
        robot.gyroHold(0.5,-90,3);
        robot.encoderDrive(10, 0.5);
        robot.gyroHold(0.5,0,3);

    }
}

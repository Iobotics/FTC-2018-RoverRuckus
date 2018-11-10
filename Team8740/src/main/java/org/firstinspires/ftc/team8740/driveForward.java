package org.firstinspires.ftc.team8740;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by student on 11/9/2018.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "driveStraight", group = "bot")
public class driveForward extends LinearOpMode {
    private Bot robot = new Bot(this);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap,false);
        waitForStart();
        robot.driveStraight(1);
    }
}

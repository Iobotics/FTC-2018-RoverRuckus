package org.firstinspires.ftc.team15076;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoTestWinch", group = "Bot")
//@Disabled
public class WinchTest extends LinearOpMode {

    public Bot robot = new Bot(this);

    @Override
    public void runOpMode() {
        /*
        Auto For the Closest To the Crater
         */

        robot.init(hardwareMap);

        waitForStart();
        robot.winchPos(1, 0.5);
    }




}

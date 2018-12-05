package org.firstinspires.ftc.team15076;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoTestEncoder", group = "Bot")
//@Disabled
public class AutoStrightTest extends LinearOpMode {

    public Bot robot = new Bot(this);

    @Override
    public void runOpMode() {
        /*
        Auto For the Closest To the Crater
         */

        robot.init(hardwareMap);

        waitForStart();
        robot.encoderDrive(10,1);

    }




}


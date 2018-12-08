package org.firstinspires.ftc.team15076;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Reid Ginoza on 9/12/2018.
 *
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="15076AutoNearCraterColor", group = "Bot")
//base auto
public class AutonomousNearCraterColor extends LinearOpMode {
    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() {


        //Robot will be backwards when operating
        robot.init(hardwareMap);

        waitForStart();

        robot.encoderDrive(10);

        if(robot.sample() == Bot.Sampling.CENTER)
        {

        }

    }
}
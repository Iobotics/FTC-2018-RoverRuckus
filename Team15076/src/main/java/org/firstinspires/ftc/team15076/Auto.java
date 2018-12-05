package org.firstinspires.ftc.team15076;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoDepotBlue", group = "Bot")
//@Disabled
public class Auto extends LinearOpMode {

    public Bot robot = new Bot(this);

    @Override
    public void runOpMode() {
        /*
        Auto For the Closest To the Crater
         */

        robot.init(hardwareMap, false);

        waitForStart();
        robot.setLiftPower(1);
        robot.setPower(-0.5, -0.5, -0.5, -0.5);
        robot.sleep(850);
        robot.setLiftPower(0);
        robot.driveStraight(-10);
        robot.gyroTurn(-90);
        robot.driveStraight(18);
        robot.gyroTurn(330);
        robot.driveStraight(24);
        robot.driveStraight(-30);
        robot.gyroTurn(180);
        robot.setExtPower(1);
        robot.sleep(1000);
        robot.setExtPower(0);

        }




    }


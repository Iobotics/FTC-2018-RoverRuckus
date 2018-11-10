package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Auto", group = "Bot")
//@Disabled
public class Auto extends LinearOpMode {

    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        /*
        Auto For the Closest To the Crater
         */
        robot.init(hardwareMap);

        waitForStart();
        robot.setLiftPower(1);
        robot.sleep(1300);
        robot.setLiftPower(0);
        robot.driveStraight(-15);
        robot.gyroTurn(-90);
        robot.driveStraight(40);
        robot.gyroTurn(135);
        robot.driveStraight(-50);
        robot.setServo(0);
        robot.driveStraight(108);




        /*waitForStart();
        robot.setLiftPower(-1);
        robot.driveStraight(-10);
        robot.gyroTurn(-90);
        robot.driveStraight(-6);
        robot.driveStraight(3);
        if (robot.isYellow()) {
            robot.gyroTurn(-135);
            robot.driveStraight(15);
            robot.setServo(0);
            robot.gyroTurn(0);
            robot.driveStraight(108);
        }
        robot.driveStraight(3);
        if (robot.isYellow()) {
            robot.gyroTurn(-150);
            robot.driveStraight(12);
            robot.setServo(0);
            robot.gyroTurn(60);
            robot.driveStraight(108);
        }
        robot.driveStraight(3);
        if (robot.isYellow()) {
            robot.gyroTurn(-170);
            robot.driveStraight(10);
            robot.setServo(0);
            robot.gyroTurn(80);
            robot.driveStraight(108);*/
        }




    }


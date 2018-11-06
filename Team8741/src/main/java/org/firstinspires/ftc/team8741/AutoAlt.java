package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoAlt", group = "Bot")
//@Disabled
public class

AutoAlt extends LinearOpMode {

    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        robot.setLiftPower(1);
        robot.sleep(1000);
        robot.setLiftPower(0);
        robot.driveStraight(-15);
        robot.gyroTurn(-90);
        robot.driveStraight(21);
        robot.gyroTurn(-180);
        robot.driveStraight(15);
        robot.gyroTurn(135);
        robot.driveStraight(15);
        robot.setServo(0);
        robot.gyroTurn(45);
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

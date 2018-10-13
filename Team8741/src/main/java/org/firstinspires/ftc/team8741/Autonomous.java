package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group = "Bot")
//@Disabled
public class Autonomous extends LinearOpMode {

    private Bot robot = new Bot(this);
    double x = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        robot.setLiftPower(-1);
        robot.driveStraight(-10);
        robot.gyroTurn(-90);
        robot.driveStraight(x);
       if (robot.isYellow()) {
        robot.gyroTurn(45);
        robot.driveStraight(x);
       }
       robot.driveStraight(x);
        if (robot.isYellow()) {
            robot.gyroTurn(80);
            robot.driveStraight(x);
        }
        robot.driveStraight(x);
        if (robot.isYellow()) {
           robot.gyroTurn(135);
            robot.driveStraight(x);
        }
        robot.setServo(x);
        robot.gyroTurn(x);
        robot.driveStraight(x);



    }
}

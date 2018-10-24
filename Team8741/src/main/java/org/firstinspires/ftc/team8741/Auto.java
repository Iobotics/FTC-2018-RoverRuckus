package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Auto", group = "Bot")
//@Disabled
public class Auto extends LinearOpMode {

    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        robot.setLiftPower(-1);
        robot.driveStraight(-15);
        robot.gyroTurn(90);
        robot.driveStraight(20);
        robot.gyroTurn(180);
        robot.driveStraight(2);
        robot.gyroTurn(-90);
        robot.driveStraight(46);
        robot.gyroTurn(-135);
        robot.driveStraight(47);
        robot.gyroTurn(45);
        robot.driveStraight(108);


    }
}

package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous3", group = "Bot")
//@Disabled
public class Auto3 extends LinearOpMode {

    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        robot.driveStraight(-15);
        robot.gyroTurn(90);
        robot.driveStraight(25.75);
        robot.gyroTurn(180);
        robot.driveStraight(2);
        robot.gyroTurn(-90);
        robot.driveStraight(45.75);
        robot.gyroTurn(-135);
        robot.driveStraight(46.75);
        robot.gyroTurn(45);
        robot.driveStraight(90);


    }
}

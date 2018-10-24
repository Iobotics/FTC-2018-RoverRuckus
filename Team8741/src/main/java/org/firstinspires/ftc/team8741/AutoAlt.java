package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoAlt", group = "Bot")
//@Disabled
public class AutoAlt extends LinearOpMode {

    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        robot.setLiftPower(-1);
        robot.driveStraight(-15);
        robot.gyroTurn(-90);
        robot.driveStraight(20);
        robot.gyroTurn(-180);
        robot.driveStraight(5);
        robot.gyroTurn(90);
        robot.driveStraight(54);
        robot.gyroTurn(0);
        robot.driveStraight(108);
    }
}
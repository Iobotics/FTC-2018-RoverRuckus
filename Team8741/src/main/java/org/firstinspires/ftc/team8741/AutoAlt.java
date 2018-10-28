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

    }
}
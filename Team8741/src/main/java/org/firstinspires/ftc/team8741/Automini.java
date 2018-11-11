package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Automini", group = "Bot")
//@Disabled
public class Automini extends LinearOpMode {

    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        /*
        Auto For the Closest To the Crater
         */
        robot.init(hardwareMap);

        waitForStart();
        robot.setLiftPower(1);
        robot.setPower(-0.10, -10);
        robot.sleep(4600);
        robot.setLiftPower(0);
        robot.gyroTurn(0);
        robot.driveStraight(-17);
        robot.gyroTurn(-90);
        robot.driveStraight(42);
        robot.gyroTurn(135);
        robot.driveStraight(-45);
        robot.setServo(0);
        robot.sleep(500);
    }
}
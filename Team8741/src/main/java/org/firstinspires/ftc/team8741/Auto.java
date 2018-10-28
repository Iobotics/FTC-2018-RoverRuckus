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

        robot.driveStraight(-15);
        robot.gyroTurn(-90);
        robot.driveStraight(35);
        robot.gyroTurn(-45);
        robot.driveStraight(55);
        robot.setServo(0);
        robot.driveStraight(-108);



    }
}

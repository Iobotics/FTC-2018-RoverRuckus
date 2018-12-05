package org.firstinspires.ftc.team15076;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutonomousTest", group = "Bot")
//@Disabled
public class AutoTest extends LinearOpMode {

    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, false);

        waitForStart();

        robot.driveStraight(20);

        robot.gyroTurn(90);

        robot.driveStraight(20);
    }
}

package org.firstinspires.ftc.teamCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group = "Bot")
//@Disabled
public class Autonomous extends LinearOpMode {

    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        robot.driveStraight(43);
        robot.gyroTurn(90);
        robot.driveStraight(52);
        robot.gyroTurn(180);
        robot.driveStraight(52);
        robot.gyroTurn(-134);
        robot.driveStraight(54);
        robot.gyroTurn(-89);
        robot.driveStraight(104);
        robot.gyroTurn(-44);
        robot.driveStraight(54);
        robot.gyroTurn(1);
        robot.driveStraight(52);
        robot.gyroTurn(90);
        robot.driveStraight(52);
        robot.gyroTurn(180);
        robot.driveStraight(43);

    }
}

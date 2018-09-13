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
        robot.driveStraight(40);
        robot.gyroTurn(90);
        robot.driveStraight(28);
        robot.gyroTurn(90);
        robot.driveStraight(61);
        robot.gyroTurn(90);
        robot.driveStraight(56);
        robot.gyroTurn(90);
        robot.driveStraight(61);
        robot.gyroTurn(90);
        robot.driveStraight(28);
        robot.gyroTurn(90);
        robot.driveStraight(40);
    }
}

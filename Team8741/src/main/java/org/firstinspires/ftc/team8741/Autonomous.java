package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group = "Bot")
//@Disabled
public class Autonomous extends LinearOpMode {

    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
       robot.driveStraight(34);
       robot.gyroTurn(-90);
       robot.driveStraight(29);
       robot.gyroTurn(0);
       robot.driveStraight(29);
       robot.gyroTurn(90);
       robot.driveStraight(34);
       robot.gyroTurn(180);
       robot.driveStraight(59);

        robot.driveStraight(43);
        robot.gyroTurn(0.4 ,90);
        robot.driveStraight(54);
        robot.gyroTurn(0.4,180);
        robot.driveStraight(54);
        robot.gyroTurn(0.4,-134);
        robot.driveStraight(54);
        robot.gyroTurn(0.4,-89);
        robot.driveStraight(108);
        robot.gyroTurn(0.4,-44);
        robot.driveStraight(54);
        robot.gyroTurn(0.4,1);
        robot.driveStraight(52);
        robot.gyroTurn(0.4,90);
        robot.driveStraight(52);
        robot.gyroTurn(0.4,180);
        robot.driveStraight(43);
    }
}

package org.firstinspires.ftc.team8898;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group = "Bot")
//@Disabled
public class Autonomous extends LinearOpMode {

    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        robot.driveStraight(36);
        robot.gyroTurn(-90);
        robot.driveStraight(43);
        robot.gyroTurn(-135);
        robot.driveStraight(47);
        robot.gyroTurn(-180);
        robot.driveStraight(17);
        robot.gyroTurn(-225);
        robot.driveStraight(53);
    }
}

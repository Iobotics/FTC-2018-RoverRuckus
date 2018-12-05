package org.firstinspires.ftc.team15076;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoDepotBasic", group = "Bot")
//@Disabled
public class AutoDepotBasic extends LinearOpMode {
    public Bot robot = new Bot(this);

    @Override
    public void runOpMode() {
        /*
        Auto For the Closest To the Crater
         */

        robot.init(hardwareMap);

        waitForStart();
        robot.winchPos(2);
        robot.liftPos(-20);
        robot.encoderDrive(15, 1);
        robot.gyroTurn(90);
        robot.encoderDrive(40, 1);
        robot.gyroTurn(125);
        robot.encoderDrive(10, 1);
        robot.winchPos(3);

    }
}

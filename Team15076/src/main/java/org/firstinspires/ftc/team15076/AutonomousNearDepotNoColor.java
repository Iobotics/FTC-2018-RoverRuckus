package org.firstinspires.ftc.team15076;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * Created by Jack Gonser on 10/1/2018.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="15AutoNearDepotNoColor", group = "bot")
public class AutonomousNearDepotNoColor extends LinearOpMode{
    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() {
        //Robot will be backwards when operating
        robot.init(hardwareMap);

        waitForStart();

        sleep(0); //Change if needed
        //robot.liftTime(450, 1);
        robot.encoderDrive(25, 1); //Add 3 inches to compensate for lift down
        //robot.liftTime(450, -1);
        robot.gyroTurn(180);
        //robot.markerdrop();
        robot.gyroTurn(45);
        robot.driveLander(-40);
        //robot.gyroTurn(40);
        //robot.driveLander(20);
    }
}

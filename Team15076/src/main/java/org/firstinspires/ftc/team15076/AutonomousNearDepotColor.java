package org.firstinspires.ftc.team15076;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * Created by Reid Ginoza on 10/1/2018.
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="15AutoRedNearDepotColor", group = "bot")
public class AutonomousNearDepotColor extends LinearOpMode{
    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() {
        //Robot will be backwards when operating
        robot.init(hardwareMap);

        waitForStart();

        robot.liftTime(450, 1);
        robot.encoderDrive(-2, 1);
        robot.liftTime(450, -1);
        robot.gyroTurn(1,90);
        /*if(robot.isBlock())//if 1st block
        {
            robot.gyroTurn(180);//hit and turn around
            robot.encoderDrive(-4,1);//go back to start
        }
        else
        {
            robot.encoderDrive(-5,1);//go to next block
            if(robot.isBlock())//if second block
            {
                robot.gyroTurn(180);//hit and turn around
                robot.encoderDrive(-2,1);//go back to start
            }
            else//go to third block
            {
                robot.encoderDrive(-5,1);//go to third block
                robot.gyroTurn(180);//hit and turn around
                robot.encoderDrive(-15,1);//go to start
            }
        }*/

        sleep(0); //Change if needed
        robot.encoderDrive(-49, 1);
        //robot.markerdrop();  //reid is badd
        robot.gyroTurn(1,-60);
        robot.driveLander(20);

    }
}

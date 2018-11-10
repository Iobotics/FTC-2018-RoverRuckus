package org.firstinspires.ftc.team15076;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

/**
 * Created by Reid Ginoza on 9/12/2018.
 *
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="15076AutoColor", group = "Bot")
//base auto
public class AutonomousColor extends LinearOpMode {
    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() {


        //Robot will be backwards when operating
        robot.init(hardwareMap);

        waitForStart();

        robot.liftTime(450, 1);
        robot.encoderDrive(3 ,1);
        robot.liftTime(450, -1);
        robot.gyroTurn(-90); //TODO fix
        sleep(0); //Change if needed
        robot.encoderDrive(7,1);
        robot.gyroTurn(-90);
        if(robot.isBlock())//if 1st block
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
        }
        robot.gyroTurn(-80);
        robot.encoderDrive(-40, 1);
        robot.markerdrop();
        robot.gyroTurn(60);
        robot.driveLander(18);


        /*robot.setPower(0.5, 0.5);
        robot.sleep(500);
        robot.setPower(0, 0);
*/
/*
        //robot.markerServo.setPosition(0);
        waitForStart();

        if (robot.colorSensor instanceof SwitchableLight) { //if you are able to change the state of
            ((SwitchableLight) robot.colorSensor).enableLight(true);
        }

        telemetry.clear();
        telemetry.log().add("START");
        telemetry.log().add("Close to gold + silver");
        telemetry.update();

        //Undeploy hook
        robot.liftUp();

        //drive to moon rocks
        robot.encoderDrive(6,0.75);//TODO measure
        robot.gyroTurn(0.5,30);
        robot.stopDrive();

        NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();
        int color = colors.toColor();
        telemetry.clear();
        telemetry.log().add("Starting First Item Scan");
        telemetry.update();

        //check if moon rock is yellow
        if (color == Color.YELLOW) {
            telemetry.log().add("First Item is Cube");
            telemetry.update();
            robot.encoderDrive(-2,0.75);
            telemetry.clear();
        } else {
            telemetry.log().add("First Item not Cube, Try Item 2");
            telemetry.update();
            robot.gyroTurn(0.5,130);
            robot.gyroTurn(0.5,90);
            robot.stopDrive();
            if (color == Color.YELLOW) {
                telemetry.log().add("Second Item is Cube");
                telemetry.update();
                robot.encoderDrive(-2,0.75);
                telemetry.clear();
            } else {
                telemetry.log().add("Second Item not Cube, Is Item 3");
                telemetry.update();
                robot.gyroTurn(0.5,130);
                robot.gyroTurn(0.5,90);
                robot.encoderDrive(-2,0.75);
                telemetry.clear();
            }

        }
        robot.encoderDrive(2,0.75);
        //robot.markerServo.setPosition(1);
        robot.gyroTurn(0.5, 45);
        robot.encoderDrive(78,0.75);

        //robot.driveStraight(); && robot.gyroTurn()
        robot.encoderDrive(65, 1);
        robot.gyroTurn(125);
        robot.encoderDrive(100, 1);
*/
    }
}

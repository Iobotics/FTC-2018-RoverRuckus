package org.firstinspires.ftc.team8740;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;


/**
 * Created by Jack Gonser on 9/12/2018.
 */

//base auto
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoBlueDepot", group = "Bot")
public class Autonomous extends LinearOpMode {
    private Bot robot = new Bot(this);

    public class hookRaiseRunnable implements Runnable {
        @Override
        public void run() {
            robot.hookRaise();
        }
    }

    public Thread hookRaiseThread = new Thread(new Runnable() {
        @Override
        public void run() {

        }
    });

    @Override
    public void runOpMode() {
        //Robot will be backwards when operating
        robot.init(hardwareMap, false);

        hookRaiseThread.start();

        //Set Marker Servo Pos
        robot.markerServo.setPosition(0);

        waitForStart();

        hookRaiseThread = new Thread(new hookRaiseRunnable());
        hookRaiseThread.start();

        while (!robot.isLiftDone) {
            idle();
        }

        robot.isLiftDone = false;

        /*if (robot.colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) robot.colorSensor).enableLight(true);
        }*/

        telemetry.clear();
        telemetry.log().add("START");
        telemetry.log().add("Close to gold + silver");
        telemetry.update();

        //drive to moon rocks
        robot.encoderDrive(6,0.75);
        robot.gyroTurn(0.5,-30);
        robot.stopDrive();

        //NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();
        //int color = colors.toColor();
        telemetry.clear();
        telemetry.log().add("Starting First Item Scan");
        telemetry.update();

        /*
        TODO: ADD Color Sensor
         */
        //check if moon rock is yellow
        /*if (color == Color.YELLOW) {
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

        }*/

        robot.encoderDrive(2,0.75);

        robot.markerServo.setPosition(1);

        robot.gyroTurn(0.5, -90);
        robot.encoderDrive(78,0.75);

    }
}
package org.firstinspires.ftc.team8740;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

/**
 * Created by student on 10/8/2018.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoRedDepot", group = "bot")
public class AutonomousRedMark extends LinearOpMode {
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

        robot.markerServo.setPosition(0);

        hookRaiseThread.start();

        waitForStart();

        hookRaiseThread = new Thread(new hookRaiseRunnable());
        hookRaiseThread.start();

        while (!robot.isLiftDone) {
            idle();
        }

        robot.isLiftDone = false;

        hookRaiseThread.interrupt();

        /*if (robot.colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) robot.colorSensor).enableLight(true);
        }*/

        telemetry.clear();
        telemetry.log().add("START");
        telemetry.log().add("Close to gold + silver");
        telemetry.update();

        //drive to moon rocks
        robot.driveStraight(8);
        robot.gyroTurn(0.5,15);


        //NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();
        //int color = colors.toColor();
        telemetry.clear();
        telemetry.log().add("Starting First Item Scan");
        telemetry.update();

        //check if moon rock is yellow
        /*if (color == Color.YELLOW) {
            telemetry.log().add("First Item is Cube");
            telemetry.update();
            robot.encoderDrive(-2,0.75);
            telemetry.clear();
        } else {
            telemetry.log().add("First Item not Cube, Try Item 2");
            telemetry.update();
            robot.gyroTurn(0.5,-130);
            robot.gyroTurn(0.5,-90);
            robot.stopDrive();
            if (color == Color.YELLOW) {
                telemetry.log().add("Second Item is Cube");
                telemetry.update();
                robot.encoderDrive(-2,0.75);
                telemetry.clear();
            } else {
                telemetry.log().add("Second Item not Cube, Is Item 3");
                telemetry.update();
                robot.gyroTurn(0.5,-130);
                robot.gyroTurn(0.5,-90);
                robot.encoderDrive(-2,0.75);
                telemetry.clear();
            }

        }*/

        robot.driveStraight(20);

        robot.markerServo.setPosition(1);

        robot.gyroTurn(0.5, 180);
        robot.driveStraight(78);

    }
}

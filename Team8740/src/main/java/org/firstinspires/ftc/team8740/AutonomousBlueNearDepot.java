package org.firstinspires.ftc.team8740;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.ftccommon.internal.RunOnBoot;


/**
 * Created by Jack Gonser on 10/1/2018.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoBlueNonDepot", group = "bot")
public class AutonomousBlueNearDepot extends LinearOpMode{
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
        robot.init(hardwareMap, true);

        robot.markerServo.setPosition(0);

        waitForStart();

        hookRaiseThread = new Thread(new hookRaiseRunnable());

        /*if (robot.colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) robot.colorSensor).enableLight(true);
        }*/

        telemetry.clear();
        telemetry.log().add("START");
        telemetry.log().add("Not near marker so must go to marker");
        telemetry.update();


        //drive from start top moon rock
        robot.gyroTurn(0.5, -45);
        robot.encoderDrive(2, 0.75);

        //drive to moon rock
        /*telemetry.log().add("At gold + silver");
        telemetry.update();
        robot.encoderDrive(-0.5,0.75);
        robot.gyroTurn(0.5,-40);*/

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
            robot.gyroTurn(0.5,130);
            robot.gyroTurn(0.5,90);
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

        robot.encoderDrive(60,0.75);

        robot.markerServo.setPosition(1);

        robot.gyroTurn(0.5, 180);
        robot.encoderDrive(78,0.75);

    }
}

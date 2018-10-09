package org.firstinspires.ftc.team8740;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

/**
 * Created by student on 10/8/2018.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoRedNonMark", group = "bot")
public class AutonomousRedNonMark extends LinearOpMode{
    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() {
        //Robot will be backwards when operating
        robot.init(hardwareMap, true);

        //robot.markerServo.setPosition(0);
        waitForStart();

        if (robot.colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) robot.colorSensor).enableLight(true);
        }

        telemetry.clear();
        telemetry.log().add("START");
        telemetry.log().add("Not near marker so must go to marker");
        telemetry.update();

        //Undeploy hook
        robot.hook.setPower(1);
        robot.sleep(3500);
        robot.hook.setPower(0);

        //drive from start top moon rock
        robot.gyroTurn(0.5, 45);
        robot.encoderDrive(-2, 0.75);
        robot.gyroTurn(0.5,0.75);

        //drive to moon rock
        telemetry.log().add("At gold + silver");
        telemetry.update();
        robot.encoderDrive(-0.5,0.75);
        robot.gyroTurn(0.5,40);

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
            robot.gyroTurn(0.5,-130);
            robot.gyroTurn(0.5,-90);
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

        }
        robot.encoderDrive(2,0.75);
        //robot.markerServo.setPosition(1);
        robot.gyroTurn(0.5, 45);
        robot.encoderDrive(78,0.75);
    }

}

package org.firstinspires.ftc.team8740;

import android.content.Context;
import android.graphics.Color;
import android.media.MediaPlayer;
import android.os.Bundle;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Jack Gonser on 8/29/2018.
 */
@TeleOp(name= "8740TeleOP", group="Bot")

public class Teleop extends LinearOpMode {
    private Bot robot = new Bot(this);

    double yValue;
    double xValue;

    double leftPower;
    double rightPower;

    public void runOpMode(){
        robot.init(hardwareMap, true); //initiate robot hardware

        telemetry.log().add("Op Mode is TELEOP"); //Visualize op mode
        telemetry.log().add("Ready For Start");
        telemetry.update(); //send to driver station
        waitForStart();
        while (opModeIsActive()) {
            //if (!gamepad1.a) {
                telemetry.clear();
                telemetry.log().clear();

                telemetry.log().add("Normal Operation");
                yValue = gamepad1.right_stick_y;
                xValue = -gamepad1.right_stick_x;

                leftPower =  yValue - xValue;
                rightPower = yValue + xValue;

                robot.setPower(Range.clip(leftPower, -1.0, 1.0),Range.clip(rightPower, -1.0, 1.0),Range.clip(leftPower, -1.0, 1.0),Range.clip(rightPower, -1.0, 1.0));

                telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
                telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);

                //raise and lower hook
                if (gamepad1.x && gamepad1.dpad_up) {
                    telemetry.log().add("Hook Up");
                    robot.hookRaise(5000);
                }
                if (gamepad1.x && gamepad1.dpad_down) {
                    telemetry.log().add("Hook Down");
                    robot.hookLower(5000);
                }

                //Move Marker Servo
                if (gamepad1.x && gamepad1.dpad_left) {
                    robot.markerServo.setPosition(0);
                    telemetry.addData("Marker Servo", "left/0");
                }
                if (gamepad1.x && gamepad1.dpad_right) {
                    robot.markerServo.setPosition(1);
                    telemetry.addData("Marker Servo","right/1");
                }

                //Quick Turn
                //left turn
                if (gamepad1.left_bumper) {
                    robot.gyroTurn(75,-45);
                }
                //right turn
                if (gamepad1.right_bumper) {
                    robot.gyroTurn(75,45);
                }

                //Intake Servo (COMP)
                if (gamepad1.y && gamepad1.dpad_up) robot.intakeServo.setPosition(0);
                if (gamepad1.y && gamepad1.dpad_down) robot.intakeServo.setPosition(0.5);

                //Individually spin motors
                if (gamepad1.right_stick_button && gamepad1.dpad_up) {
                    robot.setPower(-gamepad1.right_stick_y,0,0,0);
                } else if (gamepad1.right_stick_button && gamepad1.dpad_right) {
                    robot.setPower(0,-gamepad1.right_stick_y,0,0);
                } else if (gamepad1.right_stick_button && gamepad1.dpad_down) {
                    robot.setPower(0,0,-gamepad1.right_stick_y,0);
                } else if (gamepad1.right_stick_button && gamepad1.dpad_up) {
                    robot.setPower(0,0,0,-gamepad1.right_stick_y);
                }

                //Update Phone Log
                telemetry.update();
            }/* else {
                double speed;

                if (-gamepad1.left_stick_y > 0.4) speed = -gamepad1.left_stick_y;
                if (-gamepad1.left_stick_y <= 0.4) speed = 0.4;

                robot.intake.setPower(speed);

                NormalizedColorSensor colors = robot.intakeColor.getNormalizedColors();
                int color = colors.toColor();
                if (color = Color.YELLOW) {
                    intakeServo.setPosition(0.65);
                } else {
                    intakeServo.setPosition(0);
                }

                telemetry.log().add("INTAKE OPERATION MODE");
                telemetry.addData("Intake Speed:",speed);
                if (color = Color.YELLOW) {
                    telemetry.log().add("Cube");
                } else {
                    telemetry.log().add("Sphere");
                }
                telemetry.update();
                robot.sleep(1000);
            }
        }*/
    }
}
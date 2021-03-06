package org.firstinspires.ftc.team8740;

import android.content.Context;
import android.graphics.Color;
import android.media.MediaPlayer;
import android.os.Bundle;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Jack Gonser on 8/29/2018.
 */
@TeleOp(name= "8740TeleOP", group="Bot")

public class Teleop extends LinearOpMode {
    private Bot robot = new Bot(this);

    private double yValue;
    private double xValue;

    private double leftPower;
    private double rightPower;
    private boolean individualSpin = false;

    public void runOpMode(){
        robot.init(hardwareMap, true); //initiate robot hardware
        telemetry.addLine("Op Mode is TELEOP"); //Visualize op mode
        telemetry.addLine("Ready For Start");
        telemetry.update(); //send to driver station
        waitForStart();
        while (opModeIsActive()) {
            //if (!gamepad1.a) {

                telemetry.addLine("Normal Operation");
                yValue = gamepad1.left_stick_y;
                xValue = gamepad1.right_stick_x;

                leftPower =  yValue - xValue;
                rightPower = yValue + xValue;

                if (!individualSpin) robot.setPower(Range.clip(leftPower, -1.0, 1.0),Range.clip(rightPower, -1.0, 1.0),Range.clip(leftPower, -1.0, 1.0),Range.clip(rightPower, -1.0, 1.0));

                telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
                telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);


                 if (gamepad1.left_bumper) {
                    robot.hook.setPower(-1);
                } else  if (gamepad1.left_trigger > 0.5) {
                    robot.hook.setPower(1);
                 } else {
                     robot.hook.setPower(0);
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

                //Intake Servo (COMP)
                if (gamepad1.y && gamepad1.dpad_up) robot.intakeServo.setPosition(1);
                if (gamepad1.y && gamepad1.dpad_down) robot.intakeServo.setPosition(0);

                //Individually spin motors
                if (gamepad1.right_stick_button && gamepad1.dpad_up) {
                    robot.setPower(-gamepad1.right_stick_y,0,0,0);
                    individualSpin = true;
                } else if (gamepad1.right_stick_button && gamepad1.dpad_right) {
                    robot.setPower(0,-gamepad1.right_stick_y,0,0);
                    individualSpin = true;
                } else if (gamepad1.right_stick_button && gamepad1.dpad_down) {
                    robot.setPower(0,0,-gamepad1.right_stick_y,0);
                    individualSpin = true;
                } else if (gamepad1.right_stick_button && gamepad1.dpad_up) {
                    robot.setPower(0,0,0,-gamepad1.right_stick_y);
                    individualSpin = true;
                } else {
                    individualSpin = false;
                }

                if (gamepad1.b) {
                    telemetry.clear();
                    telemetry.addData("Right Front",robot.rightFrontDrive.getCurrentPosition());
                    telemetry.addData("Right Back",robot.rightBackDrive.getCurrentPosition());
                    telemetry.addData("Left Front",robot.leftFrontDrive.getCurrentPosition());
                    telemetry.addData("Left Back",robot.leftBackDrive.getCurrentPosition());
                    telemetry.update();
                }

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
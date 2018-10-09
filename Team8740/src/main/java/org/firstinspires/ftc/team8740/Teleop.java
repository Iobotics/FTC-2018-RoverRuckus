package org.firstinspires.ftc.team8740;

import android.graphics.Color;

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
            telemetry.clear();
            telemetry.log().clear();
            //if (!gamepad1.a) {
                telemetry.log().add("Normal Operation");
                yValue = gamepad1.right_stick_y;
                xValue = gamepad1.right_stick_x;

                leftPower =  yValue - xValue;
                rightPower = yValue + xValue;

                robot.setPower(Range.clip(leftPower, -1.0, 1.0),Range.clip(rightPower, -1.0, 1.0),Range.clip(leftPower, -1.0, 1.0),Range.clip(rightPower, -1.0, 1.0));

                telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
                telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
                telemetry.update();
                telemetry.update();

                //raise and lower hook
                if (gamepad1.x && gamepad1.dpad_up) {
                    robot.hook.setPower(1);
                }
                if (gamepad1.x && gamepad1.dpad_up) {
                    robot.hook.setPower(0);
                }

            }/* else {
                double speed = -gamepad1.left_stick_y;
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
                if () {
                    telemetry.log().add("Cube");
                } else {
                    telemetry.log().add("Sphere");
                }
                telemetry.update();
            }
        }*/
    }
}
package org.firstinspires.ftc.team15076;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Robotics on 8/29/2018.
 */
@TeleOp(name= "RunBigBoi", group="Bot")
//@Disabled
public class
Teleop extends LinearOpMode {
    private Bot robot = new Bot(this);

    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.setPower(-gamepad1.left_stick_y, -gamepad1.right_stick_y, -gamepad1.left_stick_y, -gamepad1.right_stick_y);


            if (gamepad1.left_trigger > 0.5) {
                robot.setLiftPower(-1);
            } else if (gamepad1.right_trigger > 0.5) {
                robot.setLiftPower(1);
            } else {
                robot.setLiftPower(0);
                //does lift up and down
            }
            if (gamepad1.y)//button y
            {
                robot.liftPos(29, 1);
            } else if (gamepad1.b)//button b
            {
                robot.liftPos(0, 1);
                if (gamepad1.left_bumper) {
                    robot.setInPower(-1);

                } else if (gamepad1.right_bumper) {
                    robot.setInPower(1);
                } else {
                    robot.setInPower(0);
                    //works intake and outtake
                }

                telemetry.addData("Back Left Motor Power", robot.getLeftBackPower());
                telemetry.addData("Back Right Motor Power", robot.getRightBackPower());
                telemetry.addData("Front Left Motor Power", robot.getLeftFrontPower());
                telemetry.addData("Front Right Motor Power", robot.getRightFrontPower());
                telemetry.addData("Left Bumper: ", gamepad1.left_bumper);
                telemetry.addData("Right Bumper: ", gamepad1.right_bumper);
                telemetry.addData("Arm Position", robot.getWinchPos());
                telemetry.addData("Gyro", robot.getGyroHeading());
                telemetry.addData("LeftLiftMotor", robot.leftLift.getCurrentPosition());
                telemetry.addData("RightLiftMotor", robot.rightLift.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}

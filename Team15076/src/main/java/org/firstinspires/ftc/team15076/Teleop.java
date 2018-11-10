package org.firstinspires.ftc.team15076;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Reid on 8/29/2018.
 */
@TeleOp(name= "15TeleOPTank", group="Bot")

public class Teleop extends LinearOpMode {
    private Bot robot = new Bot(this);

    double power = 0;

    public void runOpMode(){
        robot.init(hardwareMap); //initiate robot hardware

        telemetry.log().add("Op Mode is TELEOP"); //Visualize op mode
        telemetry.log().add("Ready For Start");
        telemetry.update(); //send to driver station
        telemetry.clearAll();
        waitForStart();
        while (opModeIsActive()) {
            robot.setPower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
            if(gamepad1.right_bumper)
            {
                robot.liftPower(1);
            }
            else if(gamepad1.right_trigger>=0.25)
            {
                robot.liftPower(-1);
            }
            else
            {
                robot.liftStop();
            }

            if(gamepad1.y)//button y
            {
                robot.liftTime(450, 1);
            }
            else if(gamepad1.b)//button b
            {
                robot.liftTime(450, -1);
            }
            else
            {
                robot.liftStop();
            }

            telemetry.addData("liftPos", robot.getliftPos());
            telemetry.update();

            if(gamepad1.left_bumper)
            {
                robot.setPowerIntake(1);
            }
            else if (gamepad1.left_trigger>=.25)
            {
                robot.setPowerIntake(-1);
            }
            else
            {
                robot.setPowerIntake(0);
            }

            if(gamepad1.a)
            {
                //robot.gyroTurn(90);
                //robot.gyroTurn(180);
            }
            if(gamepad1.x)
            {
                robot.gyroTurn(45);
                //robot.encoderDrive(-5, 1);
                //robot.gyroTurn(90);
                //power = power + .01;
            }
            else
            {
                //robot.setPowerDropper(0);
            }


            robot.setPower(power, power);
            telemetry.addData("power", power);
            telemetry.addData("lift distance", robot.getliftPos());
            telemetry.addData("left distance", robot.getLeft());
            telemetry.addData("right distance", robot.getRight());
            telemetry.addData("angle", robot.getGyroHeading());
            //telemetry.addData("", robot.getRed());
        }
    }

}

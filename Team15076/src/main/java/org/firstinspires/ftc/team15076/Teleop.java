package org.firstinspires.ftc.team15076;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Reid on 8/29/2018.
 */
@TeleOp(name= "15TeleOPTank", group="Bot")

public class Teleop extends LinearOpMode {
    private Bot robot = new Bot(this);

    public void runOpMode(){
        int i = 0;
        robot.init(hardwareMap); //initiate robot hardware

        telemetry.log().add("Op Mode is TELEOP"); //Visualize op mode
        telemetry.log().add("Ready For Start");
        telemetry.update(); //send to driver station
        telemetry.clearAll();
        waitForStart();
        while (opModeIsActive()) {
            robot.setPower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
            if(gamepad1.right_trigger >= 0.25)
            {
                robot.intakePower(-1);
            }
            else if(gamepad1.left_trigger>=0.25)
            {
                robot.liftPower(1);
            }
            else
            {
                robot.intakePower(0);
                robot.liftStop();
            }

            if(gamepad1.x){
                robot.liftPos(-30, 1);
            }
            if(gamepad1.y && robot.getliftPos() != 0){
                robot.liftPos(30, 1);
            }
           if(gamepad1.dpad_up )
           {
               robot.winchPower(1);
           }
           else if(gamepad1.dpad_down )
           {
               robot.winchPower(-1);
           }
           else
           {
               robot.winchPower(0);
           }

           if(gamepad1.right_bumper)
           {
               robot.intakePower(0.75);
           }
           else if (gamepad1.left_bumper)
           {
               robot.liftPower(-1);
           }






            //robot.setPower(power, power);
            //robot.liftPower(power);
            //telemetry.addData("power", power);
            telemetry.addData("is pressed", robot.isPressed());
            telemetry.addData("lift distance", robot.getliftPos());
            telemetry.addData("left distance", robot.getLeft());
            telemetry.addData("right distance", robot.getRight());
            telemetry.addData("angle", robot.getGyroHeading());
            telemetry.addData("Back Left Motor Power", robot.getLeftBackPower());
            telemetry.addData("Back Right Motor Power", robot.getRightBackPower());
            telemetry.addData("Front Left Motor Power", robot.getLeftFrontPower());
            telemetry.addData("Front Right Motor Power", robot.getRightFrontPower());
            telemetry.addData("Left Bumper: ", gamepad1.left_bumper);
            telemetry.addData("Right Bumper: ", gamepad1.right_bumper);
            telemetry.addData("Arm Position", robot.getWinchPos());
            //telemetry.addData("", robot.getRed());
            telemetry.update();
        }
    }

}

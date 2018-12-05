package org.firstinspires.ftc.team15076;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
                robot.liftPos(29,1);
            }
            else if(gamepad1.b)//button b
            {
                 robot.liftPos(0,1);
            }
            else
            {
                //robot.liftStop();
            }

           if(gamepad1.dpad_up)
           {
               robot.winchPower(1);
           }
           if(gamepad1.dpad_down)
           {
               robot.winchPower(-1);
           }
           else
           {
               robot.winchPower(0);
           }

            telemetry.addData("is pressed", robot.isPressed());
            telemetry.addData("lift distance", robot.getliftPos());
            telemetry.addData("angle", robot.getGyroHeading());
            telemetry.update();
        }
    }

}

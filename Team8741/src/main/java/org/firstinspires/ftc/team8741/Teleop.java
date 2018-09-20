package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Robotics on 8/29/2018.
 */
@TeleOp(name= "TankDrive", group="Bot")
//@Disabled
public class
Teleop extends LinearOpMode {
    private Bot robot = new Bot(this);


    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            robot.setPower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);

            if(gamepad1.a ==  true){
                robot.setLiftPower(1);
            }

           else if(gamepad1.b){
                robot.setLiftPower(-1);
            }
            else{
                robot.setLiftPower(0);
            }

            telemetry.addData("Left Position", robot.getLeftPosition());
            telemetry.addData("Right Position", robot.getRightPosition());
            telemetry.addData("Gyro", robot.getGyroHeading());
            telemetry.update();
        }
    }
}

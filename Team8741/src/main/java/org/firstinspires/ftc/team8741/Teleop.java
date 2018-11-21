package org.firstinspires.ftc.team8741;

import android.content.Context;

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


            if(gamepad1.left_trigger > 0.5){
                robot.setLiftPower(0.8);
            }

           else if(gamepad1.right_trigger > 0.5){
                robot.setLiftPower(-0.8);
            }
            else{
                robot.setLiftPower(0);
                //does lift up and down
            }

            if(gamepad1.left_bumper){
                robot.setInPower(1);

            }
            else if(gamepad1.right_bumper){
                robot.setInPower(-1);
            }
            else {
                robot.setInPower(0);
                //works intake and outtake
            }
            if(gamepad1.y) {
                robot.setServo(1);
            }
            else if(gamepad1.x) {
                robot.setServo(-1);
            }
            else {
                robot.setServo(0);
            }

            telemetry.addData("Left Position", robot.getLeftPosition());
            telemetry.addData("Right Position", robot.getRightPosition());
            telemetry.addData("Gyro", robot.getGyroHeading());
            //telemetry.addData("Red Green Blue", robot.getHSV());
            telemetry.update();
        }
    }
}
            // util.writeToFile("1", );

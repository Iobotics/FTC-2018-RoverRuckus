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


    public void runOpMode(){
        robot.init(hardwareMap, true);
        waitForStart();
        while(opModeIsActive()) {
            robot.setPower(-gamepad1.left_stick_y, -gamepad1.right_stick_y, -gamepad1.left_stick_y, -gamepad1.right_stick_y);


            if(gamepad1.left_trigger > 0.5){
                robot.setLiftPower(-1);
            }

           else if(gamepad1.right_trigger > 0.5){
                robot.setLiftPower(1);
            }
            else{
                robot.setLiftPower(0);
                //does lift up and down
            }

<<<<<<< Updated upstream
<<<<<<< Updated upstream
            if(gamepad1.y)//button y
            {
                robot.liftPos(29,1);
            }
            else if(gamepad1.b)//button b
            {
                 robot.liftPos(0,1);
=======
            if(gamepad1.left_bumper){
                robot.setInPower(-1);

            }
            else if(gamepad1.right_bumper){
                robot.setInPower(1);
>>>>>>> Stashed changes
            }
            else {
                robot.setInPower(0);
                //works intake and outtake
            }
<<<<<<< Updated upstream

<<<<<<< HEAD
           /* if(gamepad1.left_bumper)
            {
                robot.setPowerIntake(1);
=======
            if(gamepad1.dpad_up){
                robot.setExtPower(1);
            }
            else if(gamepad1.dpad_down){
                robot.setExtPower(-1);
            }
            else{
                robot.setExtPower(0);
>>>>>>> Stashed changes
            }
            if(gamepad1.x){
                robot.setServo(0.75);
            }
=======
            if(gamepad1.left_bumper){
                robot.setInPower(-1);

            }
            else if(gamepad1.right_bumper){
                robot.setInPower(1);
            }
            else {
                robot.setInPower(0);
                //works intake and outtake
            }
            if(gamepad1.dpad_up){
                robot.setExtPower(1);
            }
            else if(gamepad1.dpad_down){
                robot.setExtPower(-1);
            }
            else{
                robot.setExtPower(0);
            }
            if(gamepad1.x){
                robot.setServo(0.75);
            }
>>>>>>> Stashed changes
            else if(gamepad1.y){
                robot.setServo(0);
            }

<<<<<<< Updated upstream
<<<<<<< Updated upstream
            if(gamepad1.a)
            {
                robot.setPowerDropper(1);
            }
            if(gamepad1.x)
            {
                robot.setPowerDropper(-1);
            }
            else
            {
                robot.setPowerDropper(0);
            }*/

=======
>>>>>>> 8efe7d2740f7ddf934cca9cca3e9c80bf5b2343b
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
<<<<<<< HEAD
            //telemetry.addData("", robot.getRed());
=======
            /*
            //set position to 90 degrees
            if(gamepad1.a){
            while(position < realPos){
            motor go up
            }
            }
            //set position to 45 degrees
            if(gamepad1.b){

            }
            */



=======
            /*
            //set position to 90 degrees
            if(gamepad1.a){
            while(position < realPos){
            motor go up
            }
            }
            //set position to 45 degrees
            if(gamepad1.b){

            }
            */



>>>>>>> Stashed changes
            telemetry.addData("Gyro", robot.getGyroHeading());
            telemetry.addData("LeftLiftMotor", robot.leftLift.getCurrentPosition());
            telemetry.addData("RightLiftMotor", robot.rightLift.getCurrentPosition());
            //telemetry.addData("Red Green Blue", robot.getHSV());
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> 8efe7d2740f7ddf934cca9cca3e9c80bf5b2343b
            telemetry.update();
        }
    }
}
            // util.writeToFile("1", );

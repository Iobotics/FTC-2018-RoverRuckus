package org.firstinspires.ftc.team8898;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Robotics on 8/29/2018.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="teleop", group = "Bot")

public class Teleop extends LinearOpMode {
    private Bot robot = new Bot(this);

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        //robot.setDropper(0);


        waitForStart();
        while(opModeIsActive()) {
            robot.setPower(-gamepad1.right_stick_y, -gamepad1.left_stick_y);
            if (gamepad1.b) {
                robot.setLiftPower(.7);
            }
            else if (gamepad1.a){
                robot.setLiftPower(-.7);
            }
            else {
                robot.setLiftPower(0);
            }

            if (gamepad1.right_bumper) {
                robot.setIntakePower(1.0);
            }
            else if (gamepad1.left_bumper){
                robot.setIntakePower(-1.0);
            }
            else {
                robot.setIntakePower(0);
            }

            /**if(gamepad1.x){
               robot.setDropper(0);
           }
            else {
               robot.setDropper(0);
           }*/
            robot.setDropper(gamepad1.left_trigger);

            telemetry.addData("Red:", robot.getRed());
            telemetry.addData("Green:", robot.getGreen());
            telemetry.addData("Blue:", robot.getBlue());
            telemetry.addData("Distance:", robot.getDistance());
            telemetry.addData("isYellow", robot.isYellow());
            telemetry.addData("Error:", robot.getError(180));
            telemetry.addData("atPos:", Math.abs(robot.getError(180)) <= 2);
            telemetry.update();
            telemetry.addData("Servo", robot.getDropper());
        }
    }
}

package org.firstinspires.ftc.teamCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Robotics on 8/29/2018.
 */
@TeleOp(name= "Teleop", group="Bot")
//@Disabled
public class
Teleop extends LinearOpMode {
    private Bot robot = new Bot(this);


    public void runOpMode(){
        robot.init(hardwareMap);
        double power = 0;
        waitForStart();
        while(opModeIsActive()) {
            robot.setPower(Range.clip(gamepad1.left_stick_y + power, -1 , 1), Range.clip(gamepad1.right_stick_y + power, -1 , 1));

            if(gamepad1.a){
                power += 0.01;
            }

            if (gamepad1.b){
                power -= 0.01;
            }
            telemetry.addData("Left Position", robot.getLeftPosition());
            telemetry.addData("Right Position", robot.getRightPosition());
            telemetry.addData("Gyro", robot.getGyroHeading());
            telemetry.addData("Pwr", power);
            telemetry.addData("righ stick", gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}

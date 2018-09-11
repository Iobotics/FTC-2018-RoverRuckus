package org.firstinspires.ftc.teamCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
        waitForStart();
        while(opModeIsActive()) {
            robot.setPower(gamepad1.left_stick_y, gamepad1.right_stick_y);

            telemetry.addData("Left Position", robot.getLeftPosition());
            telemetry.addData("Right Position", robot.getRightPosition());
        }
    }
}

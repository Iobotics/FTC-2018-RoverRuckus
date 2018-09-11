package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Robotics on 8/29/2018.
 */
@TeleOp(name= "Teleop", group="Bot")

public class Teleop extends LinearOpMode {
    private Bot robot = new Bot(this);


    public void runOpMode() {

            robot.init(hardwareMap);
        while (opModeIsActive()) {
            robot.setPower(gamepad1.left_stick_y, gamepad1.right_stick_y);
            telemetry.addData("red: ", robot.getRed());
            telemetry.addData("green: ", robot.getGreen());
            telemetry.addData("blue: ", robot.getBlue());
            telemetry.addData("distance: ", robot.getDistance());
            telemetry.update();
        }
    }
}

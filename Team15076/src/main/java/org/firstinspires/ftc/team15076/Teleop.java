package org.firstinspires.ftc.team15076;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Reed on 8/29/2018.
 */
@TeleOp(name= "8740TeleOPTank", group="Bot")

public class Teleop extends LinearOpMode {
    private Bot robot = new Bot(this);

    public void runOpMode(){
        robot.init(hardwareMap, true); //initiate robot hardware

        telemetry.log().add("Op Mode is TELEOP"); //Visualize op mode
        telemetry.log().add("Ready For Start");
        telemetry.update(); //send to driver station
        waitForStart();
        while (opModeIsActive()) {
            telemetry.clear();
            telemetry.log().clear();
           robot.setPower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        }
    }
}

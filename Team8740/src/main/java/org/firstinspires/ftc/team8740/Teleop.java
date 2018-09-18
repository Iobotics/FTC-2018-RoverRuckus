package org.firstinspires.ftc.team8740;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Robotics on 8/29/2018.
 */
@TeleOp(name= "8740TeleOPArcade", group="Bot")

public class Teleop extends LinearOpMode {
    private Bot robot = new Bot(this);

    public void runOpMode(){
        robot.init(hardwareMap); //initiate robot hardware

        telemetry.addData("Start Worked","Status: GO"); //confirm start robot
        telemetry.addData("Tele OP", "Current"); //confirm telop version
        telemetry.addData("Izec is", "bad"); //izec is bad and no one likes him
        telemetry.update(); //send to driver station
        waitForStart();
        while (opModeIsActive()) {
            telemetry.clear();
            double drive = -gamepad1.left_stick_y; //drive variable
            double turn = gamepad1.right_stick_x; //turn variable

            double left = drive + turn; //left power and left turn
            double right = drive - turn; //right power and right turn
            robot.setPower(left,right,left,right); //set motor power
            telemetry.addData("left", left); // power to left
            telemetry.addData("right", right); // power to right
        }
    }
}

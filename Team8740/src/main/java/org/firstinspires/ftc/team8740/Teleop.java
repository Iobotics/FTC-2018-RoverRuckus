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
        robot.init(hardwareMap); 

        telemetry.addData("Start Worked","Status: GO");
        telemetry.addData("Tele OP", "Current");
        telemetry.addData("Izec is", "bad");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.clear();
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            double left = drive + turn;
            double right = drive - turn;
            robot.setPower(left,right,left,right);
            telemetry.addData("left", left);
            telemetry.addData("right", right);
        }
    }
}

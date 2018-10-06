package org.firstinspires.ftc.team8740;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Jack Gonser on 8/29/2018.
 */
@TeleOp(name= "8740TeleOPArcade", group="Bot")

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
            //if (!gamepad1.a) {
                telemetry.log().add("Normal Operation");
                double drive = -gamepad1.left_stick_y; //drive variable
                double turn = gamepad1.right_stick_x; //turn variable
                double driveR = 200/150;
                double driveY = (drive + 75) * driveR - 100;
                double turnR = 200/150;
                double turnY = (turn + 75) * turnR - 100;
                double left = driveY + turnY; //left power and left turn
                double right = driveY - turnY; //right power and right turn
                robot.setPower(left, right, left, right); //set motor power
                telemetry.addData("left", left); // power to left
                telemetry.addData("right", right); // power to right
                telemetry.update();

                //raise and lower hook
                if (gamepad1.x && gamepad1.dpad_up) {
                    robot.hook.setPower(1);
                }
                if (gamepad1.x && gamepad1.dpad_up) {
                    robot.hook.setPower(0);
                }
                
            /*}else {
            double speed = -gamepad1.left_stick_y;
            robot.liftOne.setPower(speed);
            telemetry.log().add("ARM OPERATION MODE");
            telemetry.addData("Arm Speed:",speed);
            telemetry.update();
            } */
        }
    }
}

package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Robotics on 8/29/2018.
 */
@TeleOp(name= "Calibrate", group="Bot")
//@Disabled
public class AutoCalibrator extends LinearOpMode {
    private Bot robot = new Bot(this);

    private double fPower = 0;
    private double timeOnPress;
    private double pCoeff_turn = 0.005;
    private double pCoeff_drive = 0.0004;

    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            robot.setPower(gamepad1.left_stick_y + fPower, gamepad1.right_stick_y + fPower);

            if(gamepad1.a && ((robot.getTime().time() - timeOnPress) > 0.2)){
                timeOnPress = robot.getTime().time();
                fPower += 0.01;
            }
            if(gamepad1.b && ((robot.getTime().time() - timeOnPress) > 0.2)){
                timeOnPress = robot.getTime().time();
                fPower -= 0.01;
            }
            if(gamepad1.x && ((robot.getTime().time() - timeOnPress) > 0.2)){
                timeOnPress = robot.getTime().time();
                pCoeff_turn += 0.001;
            }
            if(gamepad1.y && ((robot.getTime().time() - timeOnPress) > 0.2)){
                timeOnPress = robot.getTime().time();
                pCoeff_turn -= 0.001;
            }
            if(gamepad1.dpad_up && ((robot.getTime().time() - timeOnPress) > 0.2)){
                timeOnPress = robot.getTime().time();
                pCoeff_drive += 0.0001;
            }

            if(gamepad1.dpad_down&& ((robot.getTime().time() - timeOnPress) > 0.2)){
                timeOnPress = robot.getTime().time();
                pCoeff_drive -= 0.001;
            }

            if(gamepad1.dpad_left&& ((robot.getTime().time() - timeOnPress) > 0.2)){
                timeOnPress = robot.getTime().time();
                robot.driveStraight(this, 20, 0.6, pCoeff_drive);
            }

            if(gamepad1.dpad_right&& ((robot.getTime().time() - timeOnPress) > 0.2)){
                timeOnPress = robot.getTime().time();
                robot.gyroTurn(0.6, 90, 2, pCoeff_turn);
            }

            telemetry.addData("Left Position", robot.getLeftPosition());
            telemetry.addData("Right Position", robot.getRightPosition());
            telemetry.addData("Gyro", robot.getGyroHeading());
            telemetry.addData("Feed Forward", fPower);
            telemetry.addData("P Coeff Turn", pCoeff_turn);
            telemetry.addData("P Coeff Drive", pCoeff_drive);
            telemetry.update();
        }
    }
}

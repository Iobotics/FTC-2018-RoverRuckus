package org.firstinspires.ftc.teamShane;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Robotics on 8/29/2018.
 */
@TeleOp(name= "Calibrate", group="Bot")
//@Disabled
public class AutoCalibrator extends LinearOpMode {
    private Bot robot = new Bot(this);

    private double servoPosition = 0;

    public void runOpMode(){
        robot.init(hardwareMap, false);
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a){
                servoPosition += 0.01;
                robot.setServo(servoPosition);
            }
            if (gamepad1.b){
                servoPosition -= 0.01;
                robot.setServo(servoPosition);
            }
            //telemetry.addData("servoPosition", robot.getServo());
            telemetry.update();
        }
    }
}

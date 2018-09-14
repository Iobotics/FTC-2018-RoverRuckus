package org.firstinspires.ftc.teamCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



/**
 * Created by Robotics on 8/29/2018.
 */
@Autonomous(name= "TestOp", group="Bot")
//@Disabled
public class TestOp extends LinearOpMode {
    private Bot robot = new Bot(this);


    public void runOpMode(){

        robot.init(hardwareMap);
        waitForStart();
        for (int i = 0; i >= 100; i += 1) {
            robot.setPower(i * 0.01,i *0.01);
            telemetry.addData("Pwr", i);
            telemetry.update();
            try {
                wait(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        robot.stopDrive();

    }
}

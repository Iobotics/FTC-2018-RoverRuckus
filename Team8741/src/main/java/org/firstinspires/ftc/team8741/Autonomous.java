package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Robotics on 8/29/2018.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name= "Autonomous", group="Bot")

public class Autonomous extends LinearOpMode {
    private Bot robot = new Bot(this);


    public void runOpMode(){
        robot.init(hardwareMap);
        robot.turn(.5, 180);
        robot.turn(.5, -90);
    }
}

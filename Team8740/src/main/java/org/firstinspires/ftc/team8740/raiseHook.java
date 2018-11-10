package org.firstinspires.ftc.team8740;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



/**
 * Created by Jack Gonser on 10/5/2018.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Hook Reset  (USE BEFORE RUN)", group = "bot")
public class raiseHook extends LinearOpMode {
    private Bot robot = new Bot(this);

    @Override
    public void runOpMode () {
        //raise hook before comp
        robot.init(hardwareMap, false);
        waitForStart();
        robot.hook.setPower(-1);
        robot.sleep(5000);
        robot.hook.setPower(0);
    }
}

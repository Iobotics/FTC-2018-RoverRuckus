package org.firstinspires.ftc.team8740;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.ftccommon.internal.RunOnBoot;

/**
 * Created by Jack Gonser & Reid Ginoza on 9/12/2018.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="870BaseAuto", group = "Bot")

public class Autonomous extends LinearOpMode {
    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();


    }
}

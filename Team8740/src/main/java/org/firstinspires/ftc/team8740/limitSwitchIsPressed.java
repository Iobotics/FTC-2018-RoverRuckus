package org.firstinspires.ftc.team8740;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "limitSwitch", group = "Bot")
public class limitSwitchIsPressed  extends LinearOpMode {
    private Bot robot = new Bot(this);

    public void runOpMode() {
        robot.init(hardwareMap, true);

        waitForStart();
        while (opModeIsActive()) {
            //telemetry.addData("Top Limit", "%s", robot.armLimitH.isPressed());
            telemetry.addData("Bottom Limit", "%s", robot.armLimitL.isPressed());
            telemetry.update();
        }
    }
}
package org.firstinspires.ftc.team15076;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Reid on 8/29/2018.
 */
@TeleOp(name= "15TeleOPTank", group="Bot")

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
            robot.setPower(gamepad1.left_stick_y, -gamepad1.right_stick_y);
        }
        if(gamepad1.right_bumper)
        {
            robot.liftUp();
        }
        else if(gamepad1.right_trigger>=0.25)
        {
            robot.liftDown();
        }
        else
        {
            robot.liftStop();
        }

        if(gamepad1.y)//button y
        {
            robot.liftpos(30, 1);//inches needs to be changed with "telemetary" and speed is from -1 to 1
        }
        else if(gamepad1.b)//button b
        {
            robot.liftpos(0, 1);
        }
        telemetry.addData("liftPos", robot.getliftPos());
        telemetry.update();

        if(gamepad1.left_bumper)
        {
         robot.inIntake();
        }
        else if (gamepad1.left_trigger>=.25)
        {
        robot.OutIntake();
        }
    }

}

package org.firstinspires.ftc.team15076;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.ftccommon.internal.RunOnBoot;

import static java.lang.System.currentTimeMillis;

/**
 * Created by Reid Ginoza on 9/12/2018.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="15076AutoNoColor", group = "Bot")
//base auto
public class AutonomousNoColor extends LinearOpMode {
    private Bot robot = new Bot(this);



    @Override
    public void runOpMode() {


        //Robot will be backwards when operating
        robot.init(hardwareMap);

        waitForStart();

        sleep(0); //Change if needed
        robot.liftTime(450, 1);
        robot.encoderDrive(40,1);
        robot.liftTime(450, -1);
        robot.gyroTurn(-55); //TODO fix
        robot.encoderDrive(70,1);
        robot.markerdrop();
        robot.gyroTurn(-10);
        robot.encoderDrive(70, 1);
        robot.gyroTurn(30);
        robot.driveLander(10);
    }
}

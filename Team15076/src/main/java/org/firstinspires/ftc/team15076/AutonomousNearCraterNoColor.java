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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="15076AutoNearCraterNoColor", group = "Bot")
//base auto
public class AutonomousNearCraterNoColor extends LinearOpMode {
    private Bot robot = new Bot(this);



    @Override
    public void runOpMode() {


        //Robot will be backwards when operating
        robot.init(hardwareMap);

        waitForStart();

        sleep(3000); //Change if needed
        //robot.liftTime(450, 1);
        robot.encoderDrive(11,1); //Add 3 inches to compensate for lift down
        //robot.liftTime(450, -1);
        robot.gyroTurn(-73);
        robot.encoderDrive(-25, 1);
        robot.gyroTurn(-43);//-38 before
        robot.encoderDrive(-8, 1);
        robot.gyroTurn(-88);
        robot.markerdrop();
        robot.gyroTurn(-43);
        robot.encoderDrive(33, 1);
    }
}

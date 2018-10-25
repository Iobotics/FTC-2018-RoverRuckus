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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="15076AutoBlue", group = "Bot")
//base auto
public class AutonomousBlue extends LinearOpMode {
    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() {


        //Robot will be backwards when operating
        robot.init(hardwareMap, false);
        /*robot.setPower(0.5, 0.5);
        robot.sleep(500);
        robot.setPower(0, 0);
*/

        //robot.markerServo.setPosition(0);
        waitForStart();

        if (robot.colorSensor instanceof SwitchableLight) { //if you are able to change the state of
            ((SwitchableLight) robot.colorSensor).enableLight(true);
        }

        telemetry.clear();
        telemetry.log().add("START");
        telemetry.log().add("Close to gold + silver");
        telemetry.update();

        //Undeploy hook
        robot.liftUp();

        //drive to moon rocks
        robot.encoderDrive(6,0.75);//TODO measure
        robot.gyroTurn(0.5,-30);
        robot.stopDrive();

       /* NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();
        int color = colors.toColor();
        telemetry.clear();
        telemetry.log().add("Starting First Item Scan");
        telemetry.update();

        //check if moon rock is yellow
        if (color == Color.YELLOW) {
            telemetry.log().add("First Item is Cube");
            telemetry.update();
            robot.encoderDrive(-2,0.75);
            telemetry.clear();
        } else if{
            telemetry.log().add("First Item not Cube, Try Item 2");
            telemetry.update();            robot.gyroTurn(0.5,130);
            robot.gyroTurn(0.5,90);
            robot.stopDrive();
            if (color == Color.YELLOW) {
                telemetry.log().add("Second Item is Cube");
                telemetry.update();
                robot.encoderDrive(-2,0.75);
                telemetry.clear();
            } else {
                telemetry.log().add("Second Item not Cube, Is Item 3");
                telemetry.update();
                robot.gyroTurn(0.5,130);
                robot.gyroTurn(0.5,90);
                robot.encoderDrive(-2,0.75);
                telemetry.clear();
            }

        }*/

        robot.encoderDrive(2,0.75);
        //robot.markerServo.setPosition(1);
        robot.gyroTurn(0.5, -45);
        robot.encoderDrive(78,0.75);

        //robot.driveStraight(); && robot.gyroTurn()
        robot.encoderDrive(65, 1);
        robot.gyroTurn(125);
        robot.encoderDrive(100, 1);

    }
}

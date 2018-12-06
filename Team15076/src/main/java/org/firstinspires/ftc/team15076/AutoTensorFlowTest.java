package org.firstinspires.ftc.team15076;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoTensorFlowTest", group = "concept")

public class AutoTensorFlowTest extends LinearOpMode {

    public Bot robot = new Bot(this);

    private Bot.Sampling sample;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();
        // robot.winchPos(2);
        robot.liftPos(20);

        sample = robot.sample();
        if (sample == Bot.Sampling.LEFT) {
            robot.winchPos(3);
            robot.liftPos(0);
            robot.intakePower(1);
        }
        else if (sample == Bot.Sampling.RIGHT) {
            robot.encoderDrive(5, 1);
            robot.gyroTurn(-35);
            robot.winchPos(3);
            robot.liftPos(0);
            robot.intakePower(1);
            robot.encoderDrive(5, 1);
            robot.intakePower(0);
            robot.liftPos(15);
            robot.encoderDrive(-5, 1);
            robot.gyroTurn(90);
            robot.encoderDrive(25, 1);
            robot.gyroTurn(180);
            robot.encoderDrive(10, 1);
            robot.gyroTurn(90);
            robot.intakePower(1);
            robot.liftPos(0);
            robot.encoderDrive(10, 1);
            robot.encoderDrive(-5, 1);
            robot.intakePower(0);
            robot.liftPos(0);
            robot.gyroTurn(0);
            robot.encoderDrive(15, 1);
            robot.gyroTurn(-45);
            robot.encoderDrive(10, 1);
        }
        else if (sample == Bot.Sampling.CENTER) {
            robot.winchPos(3);
            robot.liftPos(0);
            robot.intakePower(1);
            robot.encoderDrive(12, 1);
            robot.encoderDrive(-7, 1);
            robot.intakePower(0);
            robot.liftPos(15);
            robot.gyroTurn(90);
            robot.encoderDrive(25, 1);
            robot.gyroTurn(180);
            robot.encoderDrive(15, 1);
            robot.gyroTurn(90);
            robot.intakePower(1);
            robot.liftPos(0);
            robot.encoderDrive(5, 1);
            robot.intakePower(0);
            robot.liftPos(15);
            robot.encoderDrive(-5, 1);
            robot.gyroTurn(0);
            robot.encoderDrive(15, 1);
            robot.gyroTurn(-45);
            robot.encoderDrive(15, 1);
            robot.winchPos(0);
        }
    }
}

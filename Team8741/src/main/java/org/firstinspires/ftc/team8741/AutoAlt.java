package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoAlt", group = "Bot")
//@Disabled
public class

AutoAlt extends LinearOpMode {

    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() throws InterruptedException {

        /*
        Auto for the Closest To the Drop Off Zone
         */

        robot.init(hardwareMap);
        robot.initTFOD();

        waitForStart();
        robot.setLiftPower(1);
        robot.setPower(-0.10, -0.10);
        robot.sleep(4200);
        robot.setLiftPower(0);
        robot.gyroTurn(0);
       // robot.driveStraight(-2);
        //robot.setLiftPower(-1);
        double goldPos = robot.getGoldPos(4.2);
      //  robot.setLiftPower(0);
        if(goldPos == 2){
            robot.gyroTurn(-30);
            robot.driveStraight(-38);
            robot.driveStraight(36);
            robot.gyroTurn(0);
        }
        else if (goldPos ==1){
            robot.driveStraight(-25);
            robot.driveStraight(25);
        }

        else if(goldPos ==0) {
            robot.gyroTurn(30);
            robot.driveStraight(-38);
            robot.driveStraight(36);
            robot.gyroTurn(0);
        }
        robot.driveStraight(-17);
        robot.gyroTurn(90);
        robot.driveStraight( 53);
        robot.gyroTurn(45);
        robot.driveStraight(-59, 1);
        robot.setServo(0);
        robot.gyroTurn(45);
        robot.driveStraight(96, 1);
        }
    }

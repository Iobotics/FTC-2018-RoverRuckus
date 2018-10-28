package org.firstinspires.ftc.team8898;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group = "Bot")
//@Disabled
public class Autonomous extends LinearOpMode {

    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.setDropper(.50);
        waitForStart();
        //\robot.dropForPosition(3, -.7);
        // Moving
        robot.driveStraight(15);
        robot.gyroTurn(-90);
        robot.driveStraight(27);
        robot.gyroTurn(-140);
        robot.driveStraight(40, .7);
        //robot.gyroTurn(-181);
        // clAIm

        robot.setDropper(0);
        try {
            Thread.sleep(2000);
        }
        catch (Exception e){}
        // Sampling
        //if (robot.isYellow()){

        //}
   /*     else{
            robot.gyroTurn(-225);
            robot.driveStraight(8);
            robot.gyroTurn(-160);
            if (robot.isYellow()){

            }
            else{
                    robot.gyroTurn(-225);
                    robot.driveStraight(8);
                    robot.gyroTurn(-160);
                   if (robot.isYellow()){

                   }
                    else{
                        robot.gyroTurn(90);
                    }
            }
        }*/
        //robot.upElevator(.7, 10);
        //robot.setIntakePower(-.8);
        //wait(1000);
        //robot.setIntakePower(0);

        // Parking
        //robot.driveStraight(24);
        //robot.gyroTurn(-223);
        robot.driveStraight(-75);
    }
}

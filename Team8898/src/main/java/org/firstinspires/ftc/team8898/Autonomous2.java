package org.firstinspires.ftc.team8898;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous2", group = "Bot")
//@Disabled
public class Autonomous2 extends LinearOpMode {

    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.setDropper(.2549);
        waitForStart();
        // Moving
        robot.driveStraight(10);
        robot.gyroTurn(88);
        robot.driveStraight(30);
        robot.gyroTurn(-48);
        robot.driveStraight(47);
        // clAIm

        robot.setDropper(.15);
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
        for(int i=0; i<3; i++){
            robot.driveStraight( -20);
        }
    }
}

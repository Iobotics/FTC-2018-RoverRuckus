package org.firstinspires.ftc.team8741;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutonomousTest", group = "Bot")
//@Disabled
public class AutoTest extends LinearOpMode {

    private Bot robot = new Bot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        double goldPos = robot.getGoldPos(10);
        if(goldPos == 2){
            robot.gyroTurn(-30);
            robot.driveStraight(-35);
        }
        else if (goldPos ==1){
            robot.driveStraight(-35);
        }

        else if(goldPos ==0) {
            robot.gyroTurn(30);
            robot.driveStraight(-35);
        }
        else {}
    }
}

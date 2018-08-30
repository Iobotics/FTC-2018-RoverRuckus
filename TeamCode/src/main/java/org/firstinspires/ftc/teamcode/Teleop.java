package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Robotics on 8/29/2018.
 */
@TeleOp(name= "Teleop", group="Bot")
//@Disabled
public class Teleop {
    private DcMotor leftBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive  = null;
    private DcMotor rightBackDrive = null;
    private LinearOpMode opMode = null;
    private HardwareMap hwMap = null;

    public Bot (LinearOpMode opMode){this.opMode = opMode;}

    public void init (HardwareMap ahwMap) {
        hwMap = ahwMap;

        leftBackDrive = hwMap.get(DcMotor.class, "backLeft");
        leftFrontDrive = hwMap.get(DcMotor.class, "frontLeft");
        rightBackDrive = hwMap.get(DcMotor.class, "backRight");
        rightFrontDrive = hwMap.get(DcMotor.class, "frontRight");
    }
}

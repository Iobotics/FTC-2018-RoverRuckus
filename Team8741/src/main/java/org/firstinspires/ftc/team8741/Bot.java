/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.team8741;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

public class Bot {

    final static int ENCODER_TICKS_PER_REV = 1120;
    final static int WHEEL_DIAMETER = 4; //Inches
    final static double INCHES_PER_TICK = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV;

    int _leftOffset;
    int _rightOffset;

    private DcMotor leftBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive  = null;
    private DcMotor rightBackDrive = null;
    private LinearOpMode opMode = null;
    private HardwareMap hwMap = null;
    private BNO055IMU imu = null;
    private ColorSensor color = null;
    private DistanceSensor distance = null;


    private Orientation angles = null;
    private Acceleration gravity = null;

    public Bot (LinearOpMode opMode){this.opMode = opMode;}

    public void init (HardwareMap ahwMap) {
        hwMap = ahwMap;

        leftBackDrive = hwMap.get(DcMotor.class, "backLeft");
        leftFrontDrive = hwMap.get(DcMotor.class, "frontLeft");
        rightBackDrive = hwMap.get(DcMotor.class, "backRight");
        rightFrontDrive = hwMap.get(DcMotor.class, "frontRight");
        color = hwMap.get(ColorSensor.class, "color");
        distance = hwMap.get(DistanceSensor.class, "distance");

        initGyro();
        setLeftDirection(DcMotor.Direction.REVERSE);
    }
    public void initGyro() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void setPower(double leftPower, double rightPower){
        leftBackDrive.setPower(leftPower);
        leftFrontDrive.setPower(leftPower);
        rightBackDrive.setPower(rightPower);
        rightFrontDrive.setPower(rightPower);
    }

    public void encoderDrive(double power, int distance) {
        while (distance < leftBackDrive.getCurrentPosition()) {
            leftBackDrive.setPower(power);
            leftFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
            rightFrontDrive.setPower(power);
        }
    }

    public double getGyroHeading() {
        // Update gyro
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();

        double heading = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        return heading;
    }

    public void turn(double power, double degrees){
        double startingDegrees = getGyroHeading();
        double degreesToMove = startingDegrees + degrees;
        while(degreesToMove >180){
            degreesToMove -= 180;
        }
        while(degreesToMove < -180){
            degreesToMove += 180;
        }
        if(degreesToMove - startingDegrees < 0) {
            while (degreesToMove < getGyroHeading()-1) {
                leftBackDrive.setPower(-power);
                leftFrontDrive.setPower(-power);
                rightBackDrive.setPower(power);
                rightFrontDrive.setPower(power);
                opMode.telemetry.addData("heading:", getGyroHeading());
                opMode.telemetry.addData("degrees", degreesToMove);
                opMode.telemetry.update();
            }
        }
        if(degreesToMove - startingDegrees > 0) {
            while (degreesToMove > getGyroHeading()+1) {
                leftBackDrive.setPower(power);
                leftFrontDrive.setPower(power);
                rightBackDrive.setPower(-power);
                rightFrontDrive.setPower(-power);
                opMode.telemetry.addData("heading:", getGyroHeading());
                opMode.telemetry.addData("degrees", degreesToMove);
                opMode.telemetry.update();
            }
        }
    }
    public double getGreen(){
        return color.green();
    }
    public double getBlue(){
        return color.blue();
    }
    public double getRed(){
        return color.red();
    }
    public double getDistance(){
        return distance.getDistance(DistanceUnit.CM);
    }

    public void setLeftDirection(DcMotor.Direction direction){
        leftBackDrive.setDirection(direction);
        leftFrontDrive.setDirection(direction);
    }

    public void setRightDirection(DcMotor.Direction direction){
        rightBackDrive.setDirection(direction);
        rightFrontDrive.setDirection(direction);
    }
}
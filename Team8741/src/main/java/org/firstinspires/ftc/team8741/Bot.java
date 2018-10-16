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

import android.graphics.Color;
import android.media.MediaPlayer;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//Disabled
public class Bot {

    final static int ENCODER_TICKS_PER_REV = 1120;
    final static int WHEEL_DIAMETER = 4; //Inches

    final static double INCHES_PER_TICK = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV;
    final static int DRIVE_THRESHOLD = (int) (0.1 / INCHES_PER_TICK);


    int _leftOffset;
    int _rightOffset;

    private final static double HEADING_THRESHOLD = 1; // As tight as we can make it with an integer gyro

    private final static double P_TURN_COEFF = 0.0035;   // Larger is more responsive, but also less stable
    private final static double P_DRIVE_COEFF = 0.00045 ;  // Larger is more responsive, but also less stable
    private final static double F_MOTOR_COEFF = 0.09;   //Larger the lower the minimum motor power is
    private final static double HOLD_TIME = 0.7; //number of milliseconds the bot has to hold a position before the turn is completed

    private final static double AUTO_DRIVE_SPEED = 0.6;
    private final static double AUTO_TURN_SPEED = 0.6;
    private final static double POWER_DAMPEN = .001;
    private final static double TIMEOUT = 5;

    private final static int YELLOW_THRESHOLD = 10;
    private final static int YELLOW_LIMIT = 10;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor liftArm = null;
    private ColorSensor colorSensor = null;
    private Servo servo = null;

    private LinearOpMode opMode = null;

    private HardwareMap hwMap = null;

    private BNO055IMU imu = null;
    private Orientation angles = null;
    private Acceleration gravity = null;

    private ElapsedTime time = new ElapsedTime();
    private boolean timerStarted = false;

    public Bot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        //Instantiate motor objects
        leftDrive = hwMap.get(DcMotor.class, "left");
        rightDrive = hwMap.get(DcMotor.class, "right");
        liftArm = hwMap.get(DcMotor.class, "lift");
        colorSensor = hwMap.get (ColorSensor.class, "sensor_color");
        servo = hwMap.get (Servo.class, "servo");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        setLeftDirection(DcMotor.Direction.REVERSE);
        setBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _leftOffset = leftDrive.getCurrentPosition();
        _rightOffset = rightDrive.getCurrentPosition();
    }

    public float[] getHSV () {

        float[] hsvValues = new float[3];
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        return hsvValues;
    }

    public int getRed(){
        return colorSensor.red();
    }

    //Sets the power of both sides of the bot
    public void setPower(double leftPower, double rightPower) {
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    public void setLiftPower(double liftPower) {
        liftArm.setPower(liftPower);
    }

    public void setBehavior(DcMotor.ZeroPowerBehavior behavior){
        leftDrive.setZeroPowerBehavior(behavior);
        rightDrive.setZeroPowerBehavior(behavior);
    }

    public void stopDrive(){
        setPower(0,0);

    }

    public void setLeftDirection(DcMotor.Direction direction) {
        leftDrive.setDirection(direction);
    }

    public void setRightDirection(DcMotor.Direction direction) {
        leftDrive.setDirection(direction);
    }

    public int getRightPosition()
    {
        return rightDrive.getCurrentPosition();
    }

    public int getLeftPosition()
    {
        return leftDrive.getCurrentPosition();
    }


    public void driveStraight(double inches)
    {
        driveStraight(opMode, inches, AUTO_DRIVE_SPEED, P_DRIVE_COEFF);
    }

    /**
     * Method for driving straight
     *
     * @param inches Inches
     */

    public void driveStraight(LinearOpMode opmode, double inches, double maxSpeed, double pCoeff) {
        double speed = 0;
        int error;
        //sets the target encoder value
        int target = rightDrive.getCurrentPosition() + (int) (inches / INCHES_PER_TICK);
        //sets current gyro value
        double startHeading = getGyroHeading();
        // While the absolute value of the error is greater than the error threshold
        //adds the f value if positive or subtracts if negative
        while (opmode.opModeIsActive() && Math.abs(rightDrive.getCurrentPosition() - target) >= DRIVE_THRESHOLD) {
            error = target - rightDrive.getCurrentPosition();
            if (error * pCoeff < 0) {
                speed = Range.clip((error * pCoeff) - F_MOTOR_COEFF, -1, 0);
            } else {
                speed = Range.clip((error * pCoeff) + F_MOTOR_COEFF, 0, 1) ;
            }

            if (Math.abs(getGyroHeading() - startHeading) > 1){
                setPower(speed, speed + POWER_DAMPEN * (getGyroHeading() - startHeading));
            }
            else {setPower(speed, speed);}

            opmode.telemetry.addData("Drive Error", error);
            opmode.telemetry.addData("Drive Power", rightDrive.getPower());
            opMode.telemetry.update();
        }
        this.stopDrive();
    }



    /**
     * Checks if the gyro is calibrating
     *
     * @return isCalibrating
     */
    public boolean isGyroCalibrating() {
        boolean isCalibrating = !imu.isGyroCalibrated();

        return isCalibrating;
    }

    /**
     * Gets the heading of the gyro in degrees
     *
     * @return heading
     */
    public double getGyroHeading() {
        // Update gyro
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();

        double heading = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        return heading;
    }

    //parameter overloaders for gyroTurn
    public void gyroTurn(double angle)
    {
        gyroTurn(AUTO_TURN_SPEED, angle, TIMEOUT, P_TURN_COEFF);
    }

    public void gyroTurn(double angle, double timeOut)
    {
        gyroTurn(AUTO_TURN_SPEED, angle, timeOut, P_TURN_COEFF);
    }

    //Turns the bot using onHeading, times out if it does not finish
    public void gyroTurn(double speed, double angle, double timeOut, double pCoeff)
    {
        while(opMode.opModeIsActive() && !onHeading(speed, angle, pCoeff) && time.time() < timeOut)
        {
            opMode.telemetry.update();
        }
    }

    /**
     * Gets the pitch of the gyro in degrees
     *
     * @return pitch
     */
    public double getGyroPitch() {
        // Update gyro
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();

        double pitch = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.thirdAngle));
        return pitch;
    }

        /**
         * Perform one cycle of closed loop heading control.
         *
         * @param speed  Desired speed of turn.
         * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
         *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
         *               If a relative angle is required, add/subtract from current heading.
         * @param PCoeff Proportional Gain coefficient
         * @return onTarget
         */
    boolean onHeading ( double speed, double angle, double PCoeff){
        double error;
        double steer = 0;
        boolean onTarget = false;
        double leftSpeed = 0;
        double rightSpeed= 0;


        // determine turn power based on +/- error
        error = getError(angle);
        if(Math.abs(error) <= HEADING_THRESHOLD && time.time() >= HOLD_TIME && timerStarted) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }

        else if (Math.abs(error) <= HEADING_THRESHOLD ) {
            if (timerStarted == false) {
                time.reset();
                timerStarted = true;
                opMode.telemetry.addLine("Reset Time");
                steer = getSteer(error, PCoeff);
                rightSpeed = Range.clip(steer, -speed, speed);
                leftSpeed = -rightSpeed;
            }
            else{
                opMode.telemetry.addLine("Timer is running");
                steer = getSteer(error, PCoeff);
                rightSpeed = Range.clip(steer, -speed, speed);
                leftSpeed = -rightSpeed;
            }
        }

        else {
            steer = getSteer(error, PCoeff);
            rightSpeed = Range.clip(steer, -speed, speed);
            leftSpeed = -rightSpeed;
            timerStarted = false;
        }
       // Send desired speeds to motors
        setPower(leftSpeed, rightSpeed);

        // Display it for the driver
        opMode.telemetry.addData("Target", "%5.2f", angle);
        opMode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        opMode.telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        opMode.telemetry.addData("timer started", timerStarted);
        opMode.telemetry.addData("hold timer", time.time());
        opMode.telemetry.addData("on Target", onTarget);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * Positive error means the robot should turn LEFT (CCW) to reduce error.
     */
     public double getError ( double targetAngle){

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getGyroHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
     }

     /**
     * returns desired steering force.  +/- 1 range.  positive = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return steer
     */
     public double getSteer ( double error, double PCoeff){
        if (error * PCoeff < 0){
            return Range.clip(error * PCoeff, -1, 0) - F_MOTOR_COEFF;
        }
        else{return Range.clip(error * PCoeff, 0, 1) + F_MOTOR_COEFF;}
     }

     public void resetTimer () {
        time.reset();
     }

     public ElapsedTime getTime () {
        return time;
     }

     public boolean isYellow(){
         float[] hsvValues = new float[3];
         Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
         return (hsvValues[0] > YELLOW_THRESHOLD) && (YELLOW_LIMIT > hsvValues[0] );
     }

     public void setServo (double position){
         servo.setPosition(position);
     }

}

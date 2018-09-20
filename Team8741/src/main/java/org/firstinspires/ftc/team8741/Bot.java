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
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    private final static double PITCH_THRESHOLD = 1; // As tight as we can make it with an integer gyro

    private final static double P_TURN_COEFF = 0.005;   // Larger is more responsive, but also less stable
    private final static double P_DRIVE_COEFF = 0.0004 ;  // Larger is more responsive, but also less stable
    private final static double F_MOTOR_COEFF = 0.145;   //Larger the lower the minimum motor power is
    private final static double HOLD_TIME = 0.7; //number of milliseconds the bot has to hold a position before the turn is completed


    private final static double FLAT_PITCH = -1;    // Pitch when robot is flat on the balance stone
    private final static double BALANCE_PITCH = -8; // Pitch when robot is leaving the balance stone

    private final static double AUTO_DRIVE_SPEED = 0.6;
    private final static double AUTO_TURN_SPEED = 0.6;
    private final static double POWER_DAMPEN = .001;
    private final static double TIMEOUT = 5;

    private DcMotor leftBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

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



        leftBackDrive = hwMap.get(DcMotor.class, "backLeft");
        leftFrontDrive = hwMap.get(DcMotor.class, "frontLeft");
        rightBackDrive = hwMap.get(DcMotor.class, "backRight");
        rightFrontDrive = hwMap.get(DcMotor.class, "frontRight");

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

        _leftOffset = leftFrontDrive.getCurrentPosition();
        _rightOffset = rightFrontDrive.getCurrentPosition();
    }

    //Sets the power of both sides of the bot
    public void setPower(double leftPower, double rightPower) {
        leftBackDrive.setPower(leftPower);
        leftFrontDrive.setPower(leftPower);
        rightBackDrive.setPower(rightPower);
        rightFrontDrive.setPower(rightPower);
    }


    public void setBehavior(DcMotor.ZeroPowerBehavior behavior){
        leftBackDrive.setZeroPowerBehavior(behavior);
        leftFrontDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
    }

    public void stopDrive(){
        setPower(0,0);

    }

    public void setLeftDirection(DcMotor.Direction direction) {
        leftBackDrive.setDirection(direction);
        leftFrontDrive.setDirection(direction);
    }

    public void setRightDirection(DcMotor.Direction direction) {
        rightBackDrive.setDirection(direction);
        rightFrontDrive.setDirection(direction);
    }

    public int getRightPosition()
    {
        return rightFrontDrive.getCurrentPosition();
    }

    public int getLeftPosition()
    {
        return leftFrontDrive.getCurrentPosition();
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
        int target = rightFrontDrive.getCurrentPosition() + (int) (inches / INCHES_PER_TICK);
        //sets current gyro value
        double startHeading = getGyroHeading();
        // While the absolute value of the error is greater than the error threshold
        //adds the f value if positive or subtracts if negative
        while (opmode.opModeIsActive() && Math.abs(rightFrontDrive.getCurrentPosition() - target) >= DRIVE_THRESHOLD) {
            error = target - rightFrontDrive.getCurrentPosition();
            if (error * pCoeff < 0) {
                speed = Range.clip(error * pCoeff, -1, 0) - F_MOTOR_COEFF;
            } else {
                speed = Range.clip(error * pCoeff, 0, 1) + F_MOTOR_COEFF;
            }

            if (Math.abs(getGyroHeading() - startHeading) > 1){
                setPower(speed, speed + POWER_DAMPEN * (getGyroHeading() - startHeading));
            }
            else {setPower(speed, speed);}

            opmode.telemetry.addData("Left Position", this.getLeftPosition() * INCHES_PER_TICK);
            opmode.telemetry.addData("Right Position", this.getRightPosition() * INCHES_PER_TICK);
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
            }
            else{
                opMode.telemetry.addLine("Timer is running");
            }
        }

        else {
            steer = getSteer(error, PCoeff);
            leftSpeed = Range.clip(steer, -speed, speed);
            rightSpeed = -leftSpeed;
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
}

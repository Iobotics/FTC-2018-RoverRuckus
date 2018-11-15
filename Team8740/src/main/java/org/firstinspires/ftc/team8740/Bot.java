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

package org.firstinspires.ftc.team8740;

import android.os.Handler;
import android.os.Looper;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.TimeUnit;

import static java.lang.System.currentTimeMillis;


public class Bot {

    final static int ENCODER_TICKS_PER_REV = 1120;
    final static int WHEEL_DIAMETER = 4; //Inches
    final static double INCHES_PER_TICK = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV;

    int _leftOffset;
    int _rightOffset;

    public DcMotor leftBackDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;

    public ElapsedTime runtime = new ElapsedTime();

    //public DcMotor intake = null;

    public DcMotor hook = null;

    //public NormalizedColorSensor colorSensor = null;
    //public NormalizedColorSensor intakeColor = null;

    public Servo markerServo = null;
    public Servo intakeServo = null;

    public TouchSensor armLimitL = null;
    public TouchSensor armLimitH = null;

    private LinearOpMode opMode = null;
    private HardwareMap hwMap = null;

    private BNO055IMU imu = null;
    private Orientation angles = null;
    private Acceleration gravity = null;

    private final static double HEADING_THRESHOLD = 1; // As tight as we can make it with an integer gyro

    private final static double P_TURN_COEFF = 0.0517;   // Larger is more responsive, but also less stable
    private final static double P_DRIVE_COEFF = 0.0002;  // Larger is more responsive, but also less stable
    private final static double F_MOTOR_COEFF = 0.11;   //Minimum amount of power given to motor from control loop

    private final static double AUTO_DRIVE_SPEED = 0.6;
    private final static double AUTO_TURN_SPEED = 0.6;
    private final static double POWER_DAMPEN = .1;

    private ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    final static int DRIVE_THRESHOLD = (int) (0.1 / INCHES_PER_TICK);

    private boolean limitHitL = false;
    private boolean limitHitH = false;

    public Bot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap ahwMap, boolean teleop) {
        hwMap = ahwMap;
        //Drive Motors
        leftBackDrive = hwMap.get(DcMotor.class, "backLeft");
        leftFrontDrive = hwMap.get(DcMotor.class, "frontLeft");
        rightBackDrive = hwMap.get(DcMotor.class, "backRight");
        rightFrontDrive = hwMap.get(DcMotor.class, "frontRight");

        //intake Motors (Expected)
        //intake = hwMap.get(DcMotor.class, "Lift1");

        //Hook Motor
        hook = hwMap.get(DcMotor.class, "hook");

        //Servos (Expected)
        markerServo = hwMap.get(Servo.class, "marker");
        intakeServo = hwMap.get(Servo.class, "intake");

        //Color Sensor
        //colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");
        //intakeColor = hwMap.get(NormalizedColorSensor.class, "intake");

        //Touch Sensors
        armLimitL = hwMap.get(TouchSensor.class, "armLow");
        armLimitH = hwMap.get(TouchSensor.class, "armHigh");

        //Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Drive Config
        if (teleop) {
            leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        Looper.prepare();
    }

    /**
     * Set the drive power
     *
     * @param leftFront
     * @param rightFront
     * @param leftBack
     * @param rightBack
     */
    public void setPower(double leftFront, double rightFront, double leftBack, double rightBack) {
        leftBackDrive.setPower(leftBack);
        leftFrontDrive.setPower(leftFront);
        rightBackDrive.setPower(rightBack);
        rightFrontDrive.setPower(rightFront);
    }

    /**
     * Stop robot movement
     */
    public void stopDrive() {
        setPower(0, 0, 0, 0);

    }

    /**
     * Sleep from LinearOpMode
     *
     * @param milliseconds
     */
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Raise hook without interrupting main thread
     */
    public void hookRaise(){
        hook.setPower(0.5);
        runtime.reset();
        while (armLimitL.isPressed()) {
            opMode.telemetry.addData("Is pressed?","%s", armLimitH.isPressed());
            opMode.telemetry.update();
            opMode.idle();
        }
        /*while (!limitHitL) {
            if (armLimitL.isPressed()) {
                limitHitL = true;
            }
        }*/
        hook.setPower(0);
        limitHitL = false;
    }
    public void hookRaiseTeleOp () {
        while (opMode.gamepad1.x && !limitHitH) {
            hook.setPower(1);
        }
        hook.setPower(0);
    }
    public void raiseHook () {
        hook.setPower(1);
    }
    /**
     * Lower hook without interrupting main thread
     */
    public void hookLower() {
        hook.setPower(-0.5);
        while (armLimitL.isPressed()) {
            opMode.telemetry.addData("Is pressed?","%s", armLimitL.isPressed());
            opMode.idle();
        }
        /*while (!limitHitL) {
            if (armLimitL.isPressed()) {
                limitHitL = true;
            }
        }*/
        hook.setPower(0);
        limitHitL = false;
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
    /*
    TODO: Recalibrate Gyro
     */
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

    public void gyroTurn(double angle) {
        gyroTurn(AUTO_TURN_SPEED, angle);
    }

    public void gyroTurn(double speed, double angle) {
        while (opMode.opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
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


    public void gyroHold(double angle, double holdTime) {
        gyroHold(AUTO_TURN_SPEED, angle, holdTime);
    }


    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed or the error has reached a threshold
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opMode.opModeIsActive() && (holdTimer.time() < holdTime) && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.

            opMode.telemetry.addData("timer", holdTimer.time());
            opMode.telemetry.update();
        }

        // Stop all motion;
        stopDrive();
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
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            leftSpeed = speed * steer;
            rightSpeed = -leftSpeed;
        }

        // Send desired speeds to motors
        setPower(leftSpeed, rightSpeed, leftSpeed, rightSpeed);

        // Display it for the driver
        opMode.telemetry.addData("Target", "%5.2f", angle);
        opMode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        opMode.telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * Positive error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

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
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void resetTimer() {
        time.reset();
    }

    public ElapsedTime getTime() {
        return time;
    }

    public void encoderDrive(double inches, double maxSpeed) {

        double speed;
        int error;
        //sets the target encoder value
        int target = leftFrontDrive.getCurrentPosition() + (int) (inches / INCHES_PER_TICK);

        // While the absolute value of the error is greater than the error threshold
        while (opMode.opModeIsActive() && Math.abs(leftFrontDrive.getCurrentPosition() - target) >= DRIVE_THRESHOLD) {
            error = target - leftFrontDrive.getCurrentPosition();
            speed = Range.clip(error * P_DRIVE_COEFF, -maxSpeed, maxSpeed);

            setPower(speed, speed, speed, speed);
            opMode.telemetry.addData("speed: ", speed);
            opMode.telemetry.update();
        }
        stopDrive();
    }

    public Thread getThreadByName(String threadName) {
        for (Thread t : Thread.getAllStackTraces().keySet()) {
            if (t.getName().equals(threadName)) return t;
        }
        return null;
    }

    public void driveStraight(double inches) {
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
        int target = leftFrontDrive.getCurrentPosition() + (int) (inches / INCHES_PER_TICK);
        //sets current gyro value
        double startHeading = getGyroHeading();
        // While the absolute value of the error is greater than the error threshold
        //adds the f value if positive or subtracts if negative
        while (opmode.opModeIsActive() && Math.abs(leftFrontDrive.getCurrentPosition() - target) >= DRIVE_THRESHOLD) {
            error = target - leftFrontDrive.getCurrentPosition();
            if (error * pCoeff < 0) {
                speed = Range.clip((error * pCoeff) - F_MOTOR_COEFF, -1, 0);
            } else {
                speed = Range.clip((error * pCoeff) + F_MOTOR_COEFF, 0, 1);
            }

            if (false/*Math.abs(getGyroHeading() - startHeading) > 1*/) {
                //setPower(speed, -(speed + POWER_DAMPEN * (getGyroHeading() - startHeading)), speed, -(speed + POWER_DAMPEN * (getGyroHeading() - startHeading)));
            } else {
                setPower(-speed, speed, -speed, speed);
            }

            opmode.telemetry.addData("Drive Error", error);
            opmode.telemetry.addData("Drive Power", rightFrontDrive.getPower());
            opMode.telemetry.update();
        }
        this.stopDrive();
    }
}
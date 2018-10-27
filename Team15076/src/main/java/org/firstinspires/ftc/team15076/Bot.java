//TODO Wednesday 10/17 test everyting (color sensors, encoderdrive, marker drop servo, lift, intake (if done)) (test autonomus and teleop)
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

package org.firstinspires.ftc.team15076;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Bot {

    final static int ENCODER_TICKS_PER_REV = 1120;
    final static int WHEEL_DIAMETER = 4; //Inches TODO - check this
    final static double INCHES_PER_TICK = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV;

    double _leftOffset;
    double _rightOffset;
    private CRServo dropperservo = null;
    private DcMotor leftBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor liftFront = null;
    private DcMotor liftBack = null;
    private DcMotor hook = null;
    private DcMotor intake = null;
    public NormalizedColorSensor colorSensor = null;
    //public Servo markerServo = null;
    private LinearOpMode opMode = null;
    private HardwareMap hwMap = null;

    private BNO055IMU imu = null;
    private Orientation angles = null;
    private Acceleration gravity = null;

    private final static double HEADING_THRESHOLD = 1; // As tight as we can make it with an integer gyro
    private final static double PITCH_THRESHOLD = 1; // As tight as we can make it with an integer gyro

    private final static double P_TURN_COEFF = 0.02;   // Larger is more responsive, but also less stable
    private final static double P_DRIVE_COEFF = 0.16;  // Larger is more responsive, but also less stable


    private final static double FLAT_PITCH = -1;    // Pitch when robot is flat on the balance stone
    private final static double BALANCE_PITCH = -8; // Pitch when robot is leaving the balance stone

    private final static double AUTO_DRIVE_SPEED = 0.6;
    private final static double AUTO_TURN_SPEED = 0.6;
    private final static double POWER_DAMPEN = .1;

    private ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    final static int DRIVE_THRESHOLD = (int) (0.1 / INCHES_PER_TICK);

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
        dropperservo = hwMap.get(CRServo.class, "dropperServo");

        //Lift Motors (Expected)
        liftFront = hwMap.get(DcMotor.class, "liftFront");
        liftBack = hwMap.get(DcMotor.class, "liftBack");

        intake =hwMap.get(DcMotor.class, "Intake");

        //Hook Motor
        //hook = hwMap.get(DcMotor.class, "hook");

        //Servos (Expected)
        //markerServo = hwMap.get(Servo.class, "marker");

        //Color Sensor
        //colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");

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
            leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        liftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        liftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        _leftOffset = getLeft();
        _rightOffset = getRight();
    }

    public void setPower(double left, double right) {
        leftBackDrive.setPower(left);
        leftFrontDrive.setPower(left);
        rightBackDrive.setPower(-right);
        rightFrontDrive.setPower(-right);
        /*
        opMode.telemetry.addData("leftBackDrive", leftBackDrive.getCurrentPosition());
        opMode.telemetry.addData("leftFrontDrive", leftFrontDrive.getCurrentPosition());
        opMode.telemetry.addData("rightBackDrive", rightBackDrive.getCurrentPosition());
        opMode.telemetry.addData("rightFrontDrive", rightFrontDrive.getCurrentPosition());
        opMode.telemetry.update();
        */
    }

    public void stopDrive() {
        setPower(0, 0);

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

    public void gyroTurn(double angle) {
        gyroTurn(AUTO_TURN_SPEED, angle);
    }

    public void gyroTurn(double speed, double angle) {
        while (opMode.opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
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

            //opMode.telemetry.addData("timer", holdTimer.time());
            //opMode.telemetry.update();
        }

        // Stop all motion;
        stopDrive();
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relativgete to last gyro reset.
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
        setPower(leftSpeed, rightSpeed);

        // Display it for the driver
        //opMode.telemetry.addData("Target", "%5.2f", angle);
        //opMode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        //opMode.telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);

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
        int target = leftBackDrive.getCurrentPosition() + (int) (inches / INCHES_PER_TICK);

        // While the absolute value of the error is greater than the error threshold
        while (opMode.opModeIsActive() && Math.abs(leftBackDrive.getCurrentPosition() - target) >= DRIVE_THRESHOLD) {
            error = target - leftBackDrive.getCurrentPosition();
            speed = Range.clip(error * P_DRIVE_COEFF, -maxSpeed, maxSpeed);

            setPower(speed, speed);
            opMode.telemetry.addData("speed: ", speed);
            opMode.telemetry.update();
        }
        stopDrive();
    }

    public double getLeft()
    {
        return (leftFrontDrive.getCurrentPosition()-_leftOffset)*INCHES_PER_TICK;
    }

    public double getRight()
    {
        return -(rightFrontDrive.getCurrentPosition()-_rightOffset)*INCHES_PER_TICK;
    }

    public void driveLander(double inches) {

        //sets the target encoder value
        int target = leftBackDrive.getCurrentPosition() + (int) (inches / INCHES_PER_TICK);

        // While the absolute value of the error is greater than the error threshold
        while (opMode.opModeIsActive() && Math.abs(leftBackDrive.getCurrentPosition() - target) >= DRIVE_THRESHOLD) {

            setPower(1, 1);
        }
        stopDrive();
    }

    /*public void liftPower(double speed)
    {
        liftFront.setPower(speed);
        liftBack.setPower(speed);
    }

    public void liftUp() {
        liftFront.setPower(1);
        liftBack.setPower(1);
    }

    public void liftDown() {
        liftFront.setPower(-1);
        liftBack.setPower(-1);
    }

    public void liftStop() {
        liftFront.setPower(0);
        liftBack.setPower(0);
    }

    /**
     * @param inches
     * @param maxSpeed
     */
   /* public void liftpos(int inches, double maxSpeed)//no semicolon after creating a method
    {
        double speed;
        int error;
        //sets the target encoder value
        int target = leftBackDrive.getCurrentPosition() + (int) (inches / INCHES_PER_TICK);

        // While the absolute value of the error is greater than the error threshold
        while (opMode.opModeIsActive() && Math.abs(liftFront.getCurrentPosition() - target) >= DRIVE_THRESHOLD) {
            error = target - leftBackDrive.getCurrentPosition();
            speed = Range.clip(error * P_DRIVE_COEFF, -maxSpeed, maxSpeed);

            this.liftPower(speed);
            opMode.telemetry.addData("speed: ", speed);
            opMode.telemetry.update();
        }
    }

    public int getliftPos()
    {
        return liftFront.getCurrentPosition();
    }
*/

    public void markerdrop() {
        dropperservo.setPower(1);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e1) {
        }
        {
            dropperservo.setPower(-1);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e1) {
            }

        }
    }

    public void setPowerDropper(int power)
    {
        dropperservo.setPower(power);
    }


    public void setPowerIntake(int power)
    {
        intake.setPower(power);
    }

}
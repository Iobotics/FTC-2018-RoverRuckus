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

import android.os.Handler;
import android.os.Looper;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
<<<<<<< Updated upstream

=======
>>>>>>> Stashed changes
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.concurrent.TimeUnit;

import static java.lang.System.currentTimeMillis;


import java.util.concurrent.TimeUnit;

import static java.lang.System.currentTimeMillis;


public class Bot {

    final static int ENCODER_TICKS_PER_REV = 1120;
    final static int WHEEL_DIAMETER = 4; //Inches
    final static double INCHES_PER_TICK = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV;

<<<<<<< Updated upstream
    final static double F_LIFT_COEFF = 0; //Against Gravity
    final static int LIFT_THRESHOLD = 1;

    double _leftOffset;
    double _rightOffset;
    //private CRServo dropperservo = null;
    private DcMotor leftBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    public DcMotor winch = null;
    public DcMotor intake = null;
    public DcMotor leftLift = null;
    public DcMotor rightLift = null;
    //public NormalizedColorSensor colorSensor = null;
    //public ColorSensor colorSensor = null;



    //public DcMotor intake = null;



    //public NormalizedColorSensor colorSensor = null;
    //public NormalizedColorSensor intakeColor = null;


=======
    int _leftOffset;
    int _rightOffset;

    public DcMotor leftBackDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor extend = null;
    public DcMotor intake = null;
    public DcMotor leftLift = null;
    public DcMotor rightLift = null;

    //public DcMotor intake = null;



    //public NormalizedColorSensor colorSensor = null;
    //public NormalizedColorSensor intakeColor = null;

    public Servo intakeServo = null;
>>>>>>> Stashed changes


    private LinearOpMode opMode = null;
    private HardwareMap hwMap = null;
    //private Servo intakeServo = null;

    private TouchSensor limitSwitch = null;


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
<<<<<<< Updated upstream

    private ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "ATClfsj/////AAAAGatavarNS0Ylq023fWG1Jgh553vtJDAOos2tGaHmeax7mcLW+BKC1TW84Spw4Y0oriANTicNCtnBfPQMDLWgayG7lHY/BE+IM5O7IB6177cNPk1uXN9CuSuq2mBkQh7cScuDbOOYraxGjL6xYWYrxNwlPYsk3+fR8d/pcgHr0xw8uezm0kgeiTHEbv4ww6XEg6oKFre1LwMlyjo1cFBP2nL3IdTEGczeT08wXC3mMbVfH8NQL6f8P4/Z8baRRgbQUDs4d5WgKSgMT0RW3JgghWwQdih06VKl+x6OhEd2T0bYNIQ7Ljhg03Nvya9DysJ+qVzbcTIyM/u6IUIoXdEqqRYNQpO/q/sg6XnZZRBrri1j";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
/*
    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    //private TFObjectDetector tfod;
    //String VUFORIA_KEY = "ATClfsj/////AAAAGatavarNS0Ylq023fWG1Jgh553vtJDAOos2tGaHmeax7mcLW+BKC1TW84Spw4Y0oriANTicNCtnBfPQMDLWgayG7lHY/BE+IM5O7IB6177cNPk1uXN9CuSuq2mBkQh7cScuDbOOYraxGjL6xYWYrxNwlPYsk3+fR8d/pcgHr0xw8uezm0kgeiTHEbv4ww6XEg6oKFre1LwMlyjo1cFBP2nL3IdTEGczeT08wXC3mMbVfH8NQL6f8P4/Z8baRRgbQUDs4d5WgKSgMT0RW3JgghWwQdih06VKl+x6OhEd2T0bYNIQ7Ljhg03Nvya9DysJ+qVzbcTIyM/u6IUIoXdEqqRYNQpO/q/sg6XnZZRBrri1j";

=======

    private ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
>>>>>>> Stashed changes

    final static int DRIVE_THRESHOLD = (int) (0.1 / INCHES_PER_TICK);

    private boolean limitHitL = false;
    private boolean limitHitH = false;

    private boolean isComm = false;


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

<<<<<<< Updated upstream
        //dropperservo = hwMap.get(CRServo.class, "dropperServo");
        //intakeServo = hwMap.get(Servo.class, "intakeServo");

//Lift Motor
        leftLift = hwMap.get(DcMotor.class, "leftLift");
        rightLift = hwMap.get(DcMotor.class, "rightLift");
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        winch = hwMap.get(DcMotor.class, "winch");
        //rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); TODO- Test
        //leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //intake =hwMap.get(DcMotor.class, "Intake");




        //Lift Motor
        leftLift = hwMap.get(DcMotor.class, "leftLift");
        rightLift = hwMap.get(DcMotor.class, "rightLift");
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Intake Motor
        intake = hwMap.get(DcMotor.class, "intake");



        limitSwitch = hwMap.get(TouchSensor.class, "limitSwitch");
=======
        //Servos (Expected)
        intakeServo = hwMap.get(Servo.class, "intakeAct");

        //Lift Motor
        leftLift = hwMap.get(DcMotor.class, "leftLift");
        rightLift = hwMap.get(DcMotor.class, "rightLift");
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Intake Motor
        extend = hwMap.get(DcMotor.class, "arm");
        intake = hwMap.get(DcMotor.class, "intake");

>>>>>>> Stashed changes

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
<<<<<<< Updated upstream


        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftPower(0);

        //dropperservo.setDirection(DcMotorSimple.Direction.REVERSE);

        _leftOffset = getLeft();
        _rightOffset = getRight();

=======
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream

=======
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream

    }
=======
>>>>>>> Stashed changes

    /**
     * Moves arm w/ intake based on input
     */
    public void setExtPower(double extendPower){
        winch.setPower(extendPower);
    }


    /**
     * Moves arm w/ intake based on input
     */
    public void setExtPower(double extendPower){
        extend.setPower(extendPower);
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
     * Moves the lift on up and down based on input
     */
    public void setLiftPower(double liftPower){
        rightLift.setPower(liftPower);
        leftLift.setPower(liftPower);
    }

    /**
     * Moves intake to in or out
     */
    public void setInPower(double inPower){
        intake.setPower(inPower);
    }

<<<<<<< Updated upstream

=======
    /**
     * Sets servo position of intake servo
     */
    public void setServo(double servoPos){
        intakeServo.setPosition(servoPos);
    }
>>>>>>> Stashed changes

    /**
     * Update phone telemtery every second without interrupting loop.
     */
    public void updateTelemetry() {
        if (!isComm) {
            isComm = true;
            Handler handler = new Handler();
            handler.postDelayed(new Runnable() {
                @Override
                public void run() {
                    opMode.telemetry.update();
                }
            }, 1000);
            long previousMillis = currentTimeMillis();
            long currentMillis = 0;
            while (currentMillis - previousMillis < 1000) {
                currentMillis = currentTimeMillis();
            }
            isComm = false;
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
<<<<<<< Updated upstream
        this.stopDrive();
    }

    public double getLeft()
    {
        return (leftFrontDrive.getCurrentPosition()-_leftOffset)*INCHES_PER_TICK;
    }

    public double getRight()
    {
        return -(rightFrontDrive.getCurrentPosition()-_rightOffset)*INCHES_PER_TICK;
    }

    public void liftPower(double speed)
    {
        rightLift.setPower(speed);
        leftLift.setPower(speed);

        stopDrive();

=======
        stopDrive();
>>>>>>> Stashed changes
    }

    public Thread getThreadByName(String threadName) {
        for (Thread t : Thread.getAllStackTraces().keySet()) {
            if (t.getName().equals(threadName)) return t;
        }
        return null;
    }

<<<<<<< Updated upstream

    public void liftStop() {
        rightLift.setPower(0);
        leftLift.setPower(0);
    }

    public void liftPos(int inches)
    {
        liftPos(inches, 1);
    }

    public void liftPos(int inches, double maxSpeed)//no semicolon after creating a method
    {
        double speed = 0;
        int error;
        //sets the target encoder value
        int target = leftLift.getCurrentPosition() + (int) (inches / INCHES_PER_TICK);



    public void winchPower(double power)//do this
    {
       winch.setPower(power);//also do this to make variable
    }




/*
    public void markerdrop() {
        this.setPowerDropper(1);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e1) {}
        dropperservo.setPower(0);
    }

    public void setPowerDropper(double power)
    {
        dropperservo.setPower(power);
    }


    public void setPowerIntake(int power)
    {
        intake.setPower(power);
    }*/
/*
    public boolean isBlock()
    {
        return colorSensor.red() > 45 && colorSensor.green() > 34 && colorSensor.blue() < 100 && colorSensor.red() - colorSensor.blue() >= 10;
    }
    public int getRed()
    {
        return colorSensor.red();
    }
    public int getGreen()
    {
        return colorSensor.green();
    }
    public int getBlue()
    {
        return colorSensor.blue();
    }
    public int getAplha()
    {
        return colorSensor.alpha();
    }
*/
    /*private void setIntakeArm(int position)
    {
        intakeServo.setPosition(position);
    }*/
}

        this.stopDrive();
    }
}

=======
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
>>>>>>> Stashed changes

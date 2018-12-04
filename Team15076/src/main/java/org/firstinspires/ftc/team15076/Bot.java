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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class Bot {

    final static int ENCODER_TICKS_PER_REV = 1120;
    final static int WHEEL_DIAMETER = 4;
    final static double INCHES_PER_TICK = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV;
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
    //public DcMotor intake = null;
    public DcMotor leftLift = null;
    public DcMotor rightLift = null;
    //public NormalizedColorSensor colorSensor = null;
    //public ColorSensor colorSensor = null;
    private LinearOpMode opMode = null;
    private HardwareMap hwMap = null;
    //private Servo intakeServo = null;

    private TouchSensor limitSwitch = null;


    private BNO055IMU imu = null;
    private Orientation angles = null;
    private Acceleration gravity = null;

    private final static double HEADING_THRESHOLD = 15; // As tight as we can make it with an integer gyro
    private final static double PITCH_THRESHOLD = 1; // As tight as we can make it with an integer gyro

    private final static double P_TURN_COEFF = .01;  // Larger is more responsive, but also less stable
    private final static double P_DRIVE_COEFF = 0.002;  // Larger is more responsive, but also less stable
    private final static double P_LIFT_COEFF = .01;
    private final static double F_MOTOR_COEFF = .07; //.2 before starts at .1
    final static double POWER_DAMPEN = .001;
    static final double P_LANDER_COEFF = .0006;
    private final static double HOLD_TIME = .7;
    private final static double TIMEOUT = 5000;

    private final static double AUTO_DRIVE_SPEED = 0.6;
    private final static double AUTO_TURN_SPEED = 1;

    private ElapsedTime time = new ElapsedTime();
    private boolean timerStarted = false;
    final static int DRIVE_THRESHOLD = (int) (0.1 / INCHES_PER_TICK);

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static  final int timeTill = 0;

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
    private static final String VUFORIA_KEY = "AQj0pPH/////AAABmSrjBOi8mEYyq+f3D1eTfiBjSvauR9qamlC6RNhe4G2EpxiWBjQvXgldcaF1LMgyat0nDF7I66sGqhLbUTIULS1lbm+eEm0ogP0gNxEzXZJlCi0AjX+fUA2k2eYKBsG+Fil8g79yBcVMcXLIaUL3WCOG6ztSm8KhrSrdaDILJhD9eqn0NPHc3Bf2HDUTEaFObd7ui/McbzHln9ebXQa4uQAm9vbr579u+35OV5XA1vMRuRb7gY0slmpXHr5us09jrdcDbIyYOPN72ITyEnfE5SSZ7D9huGvaRhUTqfX+fZVgTUAhe5RoRPR/UY1JYjWhuyUHraS5oQ51D7CXTpeahiWtOhcX79kKcZ1Pz5PUyUZJ";
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /*/**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();


    public enum Sampling
    {
        LEFT,
        CENTER,
        RIGHT

    }

    public Bot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        //Drive Motors
        leftBackDrive = hwMap.get(DcMotor.class, "backLeft");
        leftFrontDrive = hwMap.get(DcMotor.class, "frontLeft");
        rightBackDrive = hwMap.get(DcMotor.class, "backRight");
        rightFrontDrive = hwMap.get(DcMotor.class, "frontRight");

//Lift Motor
        leftLift = hwMap.get(DcMotor.class, "leftLift");
        rightLift = hwMap.get(DcMotor.class, "rightLift");
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        winch = hwMap.get(DcMotor.class, "winch");

        //intake =hwMap.get(DcMotor.class, "intake");

        limitSwitch = hwMap.get(TouchSensor.class, "limitSwitch");

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

        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftPower(0);

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
        double beforeTime = time.time();
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
        else if (Math.abs(error) <= HEADING_THRESHOLD) {
            if (timerStarted == false) {
                time.reset();
                timerStarted = true;
                opMode.telemetry.addLine("Reset Time");
                steer = getSteer(error, PCoeff);
                if(steer < 0){
                    steer -= F_MOTOR_COEFF;
                }
                else{steer += F_MOTOR_COEFF;}
                rightSpeed = Range.clip(steer, -speed, speed);
                leftSpeed = -rightSpeed;
            }
            else{
                opMode.telemetry.addLine("Timer is running");
                steer = getSteer(error, PCoeff);
                if(steer < 0){
                    steer -= F_MOTOR_COEFF;
                }
                else{steer += F_MOTOR_COEFF;}
                rightSpeed = Range.clip(steer, -speed, speed);
                leftSpeed = -rightSpeed;
            }
        }

        else {
            steer = getSteer(error, PCoeff);
            if(steer < 0){
                steer -= F_MOTOR_COEFF;
            }
            else{steer += F_MOTOR_COEFF;}
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
            return Range.clip((error * PCoeff)  - F_MOTOR_COEFF, -1, 0);
        }
        else{return Range.clip((error * PCoeff) + F_MOTOR_COEFF, 0, 1) ;}
    }

    public void resetTimer() {
        time.reset();
    }

    public ElapsedTime getTime() {
        return time;
    }

    public void encoderDrive(int inches)
    {
        encoderDrive(inches, 1);
    }

    public void encoderDrive(int inches, double maxSpeed)
    {
        encoderDrive(opMode, inches, maxSpeed, P_DRIVE_COEFF);
    }

    public void driveLander(double inches) {
        encoderDrive(opMode, inches, 1, P_LANDER_COEFF);
    }

    /**
     * r
     * @param opmode
     * @param inches
     * @param maxSpeed
     * @param pCoeff
     */
    public void encoderDrive(LinearOpMode opmode, double inches, double maxSpeed, double pCoeff) {
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
                speed = Range.clip((error * pCoeff) + F_MOTOR_COEFF, 0, 1) ;
            }

            if (Math.abs(getGyroHeading() - startHeading) > 1){
                setPower(speed, speed + POWER_DAMPEN * (getGyroHeading() - startHeading));
            }
            else {setPower(speed, speed);}

            opmode.telemetry.addData("Drive Error", error);
            opmode.telemetry.addData("Drive Power", leftFrontDrive.getPower());
            opMode.telemetry.update();
        }
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
    }

    public void liftTime(int miliseconds, double speed)
    {
        liftPower(speed);
        try {
            Thread.sleep(miliseconds);
        } catch (InterruptedException e1) {}
    }

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
        //sets current gyro value
        double startHeading = getGyroHeading();
        // While the absolute value of the error is greater than the error threshold
        //adds the f value if positive or subtracts if negative
        while (opMode.opModeIsActive() && Math.abs(leftLift.getCurrentPosition() - target) >= LIFT_THRESHOLD) {
            error = target - leftLift.getCurrentPosition();
            if (error * P_LIFT_COEFF < 0) {
                speed = Range.clip((error * P_LIFT_COEFF) - F_LIFT_COEFF, -1, 0);
            } else {
                speed = Range.clip((error * P_LIFT_COEFF) + F_LIFT_COEFF, 0, 1);
            }
            liftPower(speed);

            opMode.telemetry.addData("Lift Error", error);
            opMode.telemetry.addData("Lift Power", leftLift.getPower());
            opMode.telemetry.update();

        }
        liftStop();


    }

    public void winchPower(double power)//do this
    {
       winch.setPower(power);//also do this to make variable
    }


    public int getliftPos()
    {
        return (int) (leftLift.getCurrentPosition() * INCHES_PER_TICK);
    }

    public boolean isPressed()
    {
        return limitSwitch.isPressed();
    }

    public Sampling sample()
    {

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            opMode.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        opMode.telemetry.addData(">", "Press Play to start tracking");
        opMode.telemetry.update();
        opMode.waitForStart();

        if (opMode.opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            double timeOffset = time.time();
            while (time.time() - timeOffset <= 3000) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    opMode.telemetry.addData("Gold Mineral Position", "Left");
                                    return Sampling.LEFT;
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    opMode.telemetry.addData("Gold Mineral Position", "Right");
                                    return Sampling.RIGHT;
                                } else {
                                    opMode.telemetry.addData("Gold Mineral Position", "Center");
                                    return Sampling.CENTER;
                                }
                            }
                        }
                        opMode.telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

        return Sampling.CENTER;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.

    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

    }


}
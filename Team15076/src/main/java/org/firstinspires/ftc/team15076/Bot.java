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
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


import java.util.concurrent.TimeUnit;

import static java.lang.System.currentTimeMillis;


import java.util.concurrent.TimeUnit;

import static java.lang.System.currentTimeMillis;
import java.util.List;


public class Bot {

    final static int ENCODER_TICKS_PER_REV = 1120;
    final static int WHEEL_DIAMETER = 4;
    final static int WINCH_DIAMETER = 1;
    final static double INCHES_PER_TICK = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV;
    final static double WINCH_INCH_PER_TICK = (WINCH_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV;
    final static double F_LIFT_COEFF = 0; //Against Gravity
    final static double F_WINCH_COEFF = 0;
    final static double WINCH_THRESHOLD = 1;
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


    private LinearOpMode opMode = null;
    private HardwareMap hwMap = null;

    private TouchSensor limitSwitch = null;


    private BNO055IMU imu = null;
    private Orientation angles = null;
    private Acceleration gravity = null;

    private final static double HEADING_THRESHOLD = 1; // As tight as we can make it with an integer gyro

    private final static double P_TURN_COEFF = .004;  // Larger is more responsive, but also less stable (0.001 works but slow)
    private final static double P_DRIVE_COEFF = 0.001;  // Larger is more responsive, but also less stable
    private final static double P_LIFT_COEFF = .01;
    private final static double F_MOTOR_COEFF = .09; //.2 before starts at .1
    private final static double P_WINCH_COEFF = 0.009;
    final static double POWER_DAMPEN = .005;
    static final double P_LANDER_COEFF = .0006;
    private final static double HOLD_TIME = .7;
    private final static double TIMEOUT = 5000;

    private final static double AUTO_DRIVE_SPEED = 1;
    private final static double AUTO_TURN_SPEED = 1;


    private ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private boolean timerStarted = false;
    final static int DRIVE_THRESHOLD = (int) (0.1 / INCHES_PER_TICK);


    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final int timeTill = 0;

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


    public enum Sampling {
        LEFT,
        CENTER,
        RIGHT

    }

    private boolean limitHitL = false;
    private boolean limitHitH = false;

    private boolean isComm = false;


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
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);


//Lift Motor
        leftLift = hwMap.get(DcMotor.class, "leftLift");
        rightLift = hwMap.get(DcMotor.class, "rightLift");
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        winch = hwMap.get(DcMotor.class, "winch");
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake = hwMap.get(DcMotor.class, "Intake");

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
     * Moves arm w/ intake based on input
     */
    public void setExtPower(double extendPower) {
        winch.setPower(extendPower);
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
    public void setLiftPower(double liftPower) {
        rightLift.setPower(liftPower);
        leftLift.setPower(liftPower);
    }

    /**
     * Moves intake to in or out
     */
    public void setInPower(double inPower) {
        intake.setPower(inPower);
    }


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
        this.stopDrive();
    }

    public double getLeft() {
        return (leftFrontDrive.getCurrentPosition() - _leftOffset) * INCHES_PER_TICK;
    }

    public double getRight() {
        return -(rightFrontDrive.getCurrentPosition() - _rightOffset) * INCHES_PER_TICK;
    }

    public void liftPower(double speed) {
        rightLift.setPower(speed);
        leftLift.setPower(speed);

        stopDrive();

    }

    public Thread getThreadByName(String threadName) {
        for (Thread t : Thread.getAllStackTraces().keySet()) {
            if (t.getName().equals(threadName)) return t;
        }
        return null;
    }


    public void liftStop() {
        rightLift.setPower(0);
        leftLift.setPower(0);
    }

    public void liftPos(int inches) {
        liftPos(inches, 1);
    }

    public void liftPos(int inches, double maxSpeed)//no semicolon after creating a method
    {
        double speed = 0;
        int error;
        //sets the target encoder value
        int target = leftLift.getCurrentPosition() + (int) (inches / INCHES_PER_TICK);
    }


    public void winchPower(double speed)//do this
    {
        winch.setPower(speed);//also do this to make variable
    }

    public void winchPos(int inches) {
        winchPos(inches, 1);
    }

    public void winchPos(int inches, double maxSpeed) {
        double speed = 0;
        int error;
        //sets the target encoder value
        int target = winch.getCurrentPosition() + (int) (inches / WINCH_INCH_PER_TICK);
        //sets current gyro value
        double startHeading = getGyroHeading();
        // While the absolute value of the error is greater than the error threshold
        //adds the f value if positive or subtracts if negative
        while (opMode.opModeIsActive() && Math.abs(winch.getCurrentPosition() - target) >= WINCH_THRESHOLD) {
            error = target - winch.getCurrentPosition();
            if (error * P_WINCH_COEFF < 0) {
                speed = Range.clip((error * P_WINCH_COEFF) - F_WINCH_COEFF, -1, 0);
            } else {
                speed = Range.clip((error * P_WINCH_COEFF) + F_WINCH_COEFF, 0, 1);
            }
            winchPower(speed);

            opMode.telemetry.addData("Winch Error", error);
            opMode.telemetry.addData("Winch Power", winch.getPower());
            opMode.telemetry.update();

        }


    }


    public void setLeftLift(double angle) {

    }

    public double getLeftBackPower() {
        return leftBackDrive.getPower();
    }

    public double getLeftFrontPower() {
        return leftFrontDrive.getPower();
    }

    public double getRightBackPower() {
        return rightBackDrive.getPower();
    }

    public double getRightFrontPower() {
        return rightFrontDrive.getPower();
    }

    public int getWinchPos() {
        return (int) (winch.getCurrentPosition() * WINCH_INCH_PER_TICK);
    }

    public void intakePower(double speed) {
        intake.setPower(speed);
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

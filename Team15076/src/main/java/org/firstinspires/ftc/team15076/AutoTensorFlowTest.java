package org.firstinspires.ftc.team15076;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoTensorFlowTest", group = "concept")

public class AutoTensorFlowTest extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "ATClfsj/////AAAAGatavarNS0Ylq023fWG1Jgh553vtJDAOos2tGaHmeax7mcLW+BKC1TW84Spw4Y0oriANTicNCtnBfPQMDLWgayG7lHY/BE+IM5O7IB6177cNPk1uXN9CuSuq2mBkQh7cScuDbOOYraxGjL6xYWYrxNwlPYsk3+fR8d/pcgHr0xw8uezm0kgeiTHEbv4ww6XEg6oKFre1LwMlyjo1cFBP2nL3IdTEGczeT08wXC3mMbVfH8NQL6f8P4/Z8baRRgbQUDs4d5WgKSgMT0RW3JgghWwQdih06VKl+x6OhEd2T0bYNIQ7Ljhg03Nvya9DysJ+qVzbcTIyM/u6IUIoXdEqqRYNQpO/q/sg6XnZZRBrri1j";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    public Bot robot = new Bot(this);

    @Override
    public void runOpMode() {
        initVuforia();

        robot.init(hardwareMap);

        waitForStart();
       // robot.winchPos(2);
        robot.liftPos(20);

            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();

            while (tfod != null) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
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
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    robot.winchPos(3);
                                    robot.liftPos(0);
                                    robot.intakePower(1);

                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    robot.encoderDrive(5,1);
                                    robot.gyroTurn(-35);
                                    robot.winchPos(3);
                                    robot.liftPos(0);
                                    robot.intakePower(1);
                                    robot.encoderDrive(5,1);
                                    robot.intakePower(0);
                                    robot.liftPos(15);
                                    robot.encoderDrive(-5,1);
                                    robot.gyroTurn(90);
                                    robot.encoderDrive(25,1);
                                    robot.gyroTurn(180);
                                    robot.encoderDrive(10,1);
                                    robot.gyroTurn(90);
                                    robot.intakePower(1);
                                    robot.liftPos(0);
                                    robot.encoderDrive(10,1);
                                    robot.encoderDrive(-5,1);
                                    robot.intakePower(0);
                                    robot.liftPos(0);
                                    robot.gyroTurn(0);
                                    robot.encoderDrive(15,1);
                                    robot.gyroTurn(-45);
                                    robot.encoderDrive(10,1);
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    robot.winchPos(3);
                                    robot.liftPos(0);
                                    robot.intakePower(1);
                                    robot.encoderDrive(12,1);
                                    robot.encoderDrive(-7,1);
                                    robot.intakePower(0);
                                    robot.liftPos(15);
                                    robot.gyroTurn(90);
                                    robot.encoderDrive(25,1);
                                    robot.gyroTurn(180);
                                    robot.encoderDrive(15,1);
                                    robot.gyroTurn(90);
                                    robot.intakePower(1);
                                    robot.liftPos(0);
                                    robot.encoderDrive(5,1);
                                    robot.intakePower(0);
                                    robot.liftPos(15);
                                    robot.encoderDrive(-5,1);
                                    robot.gyroTurn(0);
                                    robot.encoderDrive(15,1);
                                    robot.gyroTurn(-45);
                                    robot.encoderDrive(15,1);
                                    robot.winchPos(0);


                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }


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
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}

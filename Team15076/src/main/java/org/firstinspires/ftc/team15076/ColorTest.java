package org.firstinspires.ftc.team15076;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="colorTest", group = "Bot")
//@Disabled
public class ColorTest extends LinearOpMode {
    private ColorSensor colorSensor = null;
    @Override
    public void runOpMode() throws InterruptedException {
        float  hueSum = 0;
        float hueAvg = 0;
        colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");
        waitForStart();
        while(opModeIsActive()) {
            float[] hsvValues = new float[3];
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
            if (gamepad1.a) {
                hueSum = 0;
                hueAvg = 0;
                for (int i = 0; i < 1000; i++) {
                    hueSum += hsvValues[0];
                    hueAvg = hueSum / i;
                }
            }
            telemetry.addData("red", colorSensor.red());
            telemetry.addData("green", colorSensor.green());
            telemetry.addData("blue", colorSensor.blue());
            telemetry.addData("hue", hsvValues[0]);
            telemetry.addData("avg hue",hueAvg);
            telemetry.update();
        }
    }

}

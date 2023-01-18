package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Karen;

public class edgefollow extends LinearOpMode {

    Karen bot;
    NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new Karen(hardwareMap);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        double rotate = 0.0;

        while (true) {
            bot.moveBot(1, rotate, 0, 0.2);

            NormalizedRGBA colorsRGBA = colorSensor.getNormalizedColors();
            float[] hsv = {0, 0, 0};
            Color.colorToHSV(colorsRGBA.toColor(), hsv);

            if (hsv[0] < 100 && hsv[0] > 25) {
                rotate = -10;
            }
            else {
                rotate = 10;
            }
            telemetry.addData("HSV: ", hsv);
        }
    }
}

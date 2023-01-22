package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Karen;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class EdgeFollow_Testing extends LinearOpMode {

    Karen bot;
    NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new Karen(hardwareMap);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        double rotate = 0.0;

        boolean found = false;

//        while (!found) {
//            bot.moveBot(1, 0, 0, 0.2);
//
//            NormalizedRGBA colorsRGBA = colorSensor.getNormalizedColors();
//            float[] hsv = {0, 0, 0};
//            Color.colorToHSV(colorsRGBA.toColor(), hsv);
//
//            if (hsv[0] > 200 && hsv[0] < 280) {
//                found = true;
//            }
//        }
//
//        while (!found) {
//            bot.moveBot(1, 0, 0, 0.2);
//
//            NormalizedRGBA colorsRGBA = colorSensor.getNormalizedColors();
//            float[] hsv = {0, 0, 0};
//            Color.colorToHSV(colorsRGBA.toColor(), hsv);
//
//            if (hsv[2] > 125) {
//                found = true;
//            }
//        }
//
//        while (!found) {
//            bot.moveBot(0, 0.05, 0, 0.2);
//
//            NormalizedRGBA colorsRGBA = colorSensor.getNormalizedColors();
//            float[] hsv = {0, 0, 0};
//            Color.colorToHSV(colorsRGBA.toColor(), hsv);
//
//            if (hsv[2] < 125) {
//                found = true;
//            }
//        }

        while (true) {
//            bot.moveBot(1, rotate, 0, 0.2); // move straight and then rotate to adjust

            NormalizedRGBA colorsRGBA = colorSensor.getNormalizedColors();
            float[] hsv = {0, 0, 0};
            Color.colorToHSV(colorsRGBA.toColor(), hsv);

            telemetry.addData("H", hsv[0]);
            telemetry.addData("S", hsv[1]);
            telemetry.addData("V", hsv[2]);
            telemetry.addData("R", colorsRGBA.red);
            telemetry.addData("G", colorsRGBA.green);
            telemetry.addData("B", colorsRGBA.blue);
            telemetry.addData("A", colorsRGBA.alpha);

            if (hsv[2] < 125) { // threshold between tape and no tape for the HSV value
                rotate = -0.1;
            }
            else {
                rotate = 0.1;
            }

            wait(1);
        }
    }
}

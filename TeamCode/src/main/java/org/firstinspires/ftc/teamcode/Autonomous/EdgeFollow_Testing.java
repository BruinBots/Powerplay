package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Karen;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous()
public class EdgeFollow_Testing extends LinearOpMode {

    Karen bot;
    NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new Karen(hardwareMap);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        waitForStart();

        double rotate = 0.0;

//        boolean found = false;

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

        while (opModeIsActive()) {
            bot.moveBot(1, rotate, 0, 0.2); // move straight and then rotate to adjust

            NormalizedRGBA colorsRGBA = colorSensor.getNormalizedColors();
            float[] hsv = {0, 0, 0};
            Color.colorToHSV(colorsRGBA.toColor(), hsv);

            telemetry.addData("HSV", hsv[0] + ", " + hsv[1] + ", " + hsv[2]);
            telemetry.addData("RGBA", colorsRGBA.red + ", " + colorsRGBA.green + ", " + colorsRGBA.blue + ", " + colorsRGBA.alpha);
            telemetry.addData("rotate", rotate);
            telemetry.update();

            if (hsv[0] < 200 && hsv[0] > 120) { // threshold between tape and no tape for the RGBA value
                rotate = -0.5;
            }
            else {
                rotate = 0.5;
            }
        }
    }
}

/*
how does telemetry handle arrays??
 */
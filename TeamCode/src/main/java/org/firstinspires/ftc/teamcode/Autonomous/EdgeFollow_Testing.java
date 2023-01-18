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

        while (!found) {
            bot.moveBot(1, 0, 0, 0.2);

            NormalizedRGBA colorsRGBA = colorSensor.getNormalizedColors();
            float[] hsv = {0, 0, 0};
            Color.colorToHSV(colorsRGBA.toColor(), hsv);

            if (hsv[0] > 200 && hsv[0] < 280) {
                found = true;
            }
        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory turn = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(0, 0), Math.toRadians(90))
                .build();

        drive.followTrajectory(turn);

        while (true) {
            bot.moveBot(1, rotate, 0, 0.2);

            NormalizedRGBA colorsRGBA = colorSensor.getNormalizedColors();
            float[] hsv = {0, 0, 0};
            Color.colorToHSV(colorsRGBA.toColor(), hsv);

            if (hsv[2] < 125) {
                rotate = -0.1;
            }
            else {
                rotate = 0.1;
            }
        }
    }
}

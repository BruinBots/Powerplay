package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Karen;

public class LineFollower extends LinearOpMode {
    public void runOpMode() {
        Karen bot = new Karen(hardwareMap);
        ColorSensor colorSensor = hardwareMap.colorSensor.get("colorSensor");

        boolean currentlyRunning = true;

        while (currentlyRunning) {
            telemetry.addData("Red: ", colorSensor.red());
            telemetry.addData("Blue: ", colorSensor.blue());
            telemetry.addData("Green: ", colorSensor.green());
        }
        bot.moveBot(1,-10,0,0.5);
    }
}

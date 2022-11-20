package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Ezra: Autonomous", group="Iterative Opmode")
public class EzraAutonomous extends OpMode {

    public static double SQUARE_SIZE = -0.5;

    BigBob bot;

    public void init() {

        bot = new BigBob(hardwareMap);
    }

    public void start() {
        bot.moveBot(SQUARE_SIZE, 0, 0, 0.5);
    }

    @Override
    public void loop() {

    }
}
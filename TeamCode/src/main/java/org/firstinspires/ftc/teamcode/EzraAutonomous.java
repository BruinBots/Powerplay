package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Ezra: Autonomous", group="Iterative Opmode")
public class EzraAutonomous extends OpMode {

    public static int SQUARE_SIZE = 10;

    BigBob bot;

    public void init() {

        bot = new BigBob(hardwareMap);
    }

    public void start() {

    }

    @Override
    public void loop() {

    }
}
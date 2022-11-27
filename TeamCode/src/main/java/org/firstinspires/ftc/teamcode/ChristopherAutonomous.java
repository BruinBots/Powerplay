package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Christopher: Autonomous", group="Iterative Opmode")
public class ChristopherAutonomous extends OpMode {

    public static int SQUARE_SIZE = 10;

    BigBob bot;

    public void init() {

        bot = new BigBob(hardwareMap);
        telemetry.addData("Status", "Christopher Autonomous Initialized");
    }

    public void start() {
        bot.moveLinearSlide(BigBob.MAX_LINEAR_SLIDE_POSITION);
        bot.moveBot(SQUARE_SIZE * 2, 0, 0, 0.5);
        bot.moveBot(0, 0, SQUARE_SIZE / 2, 0.5);
        bot.moveClaw(BigBob.CLAW_OPEN);
        bot.moveBot(0, 0, SQUARE_SIZE / 2, 0.5);
        bot.moveLinearSlide(BigBob.MIN_LINEAR_SLIDE_POSITION);

    }

    @Override
    public void loop() { // loop is required

    }
}
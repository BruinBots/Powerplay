package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="Ezra: Autonomous", group="Iterative Opmode")
public class EzraAutonomous extends OpMode {

    public static double BOT_SPEED = -0.5;
    public static int squareSize = 24;
    public static double circumference = 1.91;
    public static double ticksPerRotation = 537.6;
    public static double leftFrontEncoder;

    BigBob bot;
    public void move(double squares, double drive, double rotation, double strafe, double scaleFactor) {
        leftFrontEncoder = bot.leftFrontMotor.getCurrentPosition();;
        double distance = squares * squareSize;
        double rotations = distance / circumference;
        double ticks = rotations * ticksPerRotation;
        bot.moveBot(drive, rotation, strafe, scaleFactor);
        while(leftFrontEncoder < ticks + leftFrontEncoder) {
            // nothing
        }
        stop();

    }
    public void init() {

        bot = new BigBob(hardwareMap);
        telemetry.addData("leftFrontEncoder", leftFrontEncoder);
    }

    public void start() {
        move(1, -0.05, 0, 0, 0.5);
    }

    @Override
    public void loop() {
    }
}
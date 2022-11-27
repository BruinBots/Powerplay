package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BigBob {


    // Class variables

    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;

    public DcMotorEx linearSlideMotor;
    public Servo clawServo;

    public static final double CLAW_OPEN = 0.95;
    public static final double CLAW_CLOSED = 0.65;

    public static final double CLAW_ZERO_POSITION = 0.65;

    public static final int MAX_LINEAR_SLIDE_POSITION = 1950;
    public static final int MIN_LINEAR_SLIDE_POSITION = 10;

    public static final int MEDIUM_LINEAR_SLIDE_POSITION = 1450;
    public static final int LOW_LINEAR_SLIDE_POSITION = 900;

    public static final double LINEAR_SLIDE_POWER = 0.2;
    public static final double LINEAR_SLIDE_POWER_DOWN = 0.065;

    public BigBob() {
    }
        // constructor with map
    public BigBob(HardwareMap map) {
        // Drivetrain Motors
        leftFrontMotor = map.get(DcMotorEx.class, "leftFrontMotor");
        rightFrontMotor = map.get(DcMotorEx.class, "rightFrontMotor");
        leftBackMotor = map.get(DcMotorEx.class, "leftBackMotor");
        rightBackMotor = map.get(DcMotorEx.class, "rightBackMotor");

        linearSlideMotor = map.get(DcMotorEx.class, "linearSlide");
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        clawServo = map.get(Servo.class, "clawServo");

        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void moveBot(double drive, double rotate, double strafe, double scaleFactor) {
        double[] wheelSpeeds = new double[4];
        wheelSpeeds[0] = drive + strafe + rotate;  // left front
        wheelSpeeds[1] = drive - strafe - rotate;  // right front
        wheelSpeeds[2] = drive - strafe + rotate;  // left rear
        wheelSpeeds[3] = drive + strafe - rotate;  // right rear

        // finding the greatest power value
        double maxMagnitude = Math.max(Math.max(Math.max(wheelSpeeds[0], wheelSpeeds[1]), wheelSpeeds[2]), wheelSpeeds[3]);


        // dividing everyone by the max power value so that ratios are same (check if sdk automatically clips to see if go build documentation works
        if (maxMagnitude > 1.0)
        {
            for (int i = 0; i < wheelSpeeds.length; i++)
            {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }



        // setting motor power and scaling down to preference
        leftFrontMotor.setPower(wheelSpeeds[0] * scaleFactor);
        rightFrontMotor.setPower(wheelSpeeds[1] * scaleFactor);
        leftBackMotor.setPower(wheelSpeeds[2] * scaleFactor);
        rightBackMotor.setPower(wheelSpeeds[3] * scaleFactor);
    }

    public void stop(){
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    public void moveClaw(double pos) {
        if (pos > CLAW_OPEN) {
            pos = CLAW_OPEN;
        }
        else if (pos < CLAW_CLOSED) {
            pos = CLAW_CLOSED;
        }
        clawServo.setPosition(pos);
    }

    public void moveLinearSlide(int ticks) {
        if (ticks > MAX_LINEAR_SLIDE_POSITION) {
            ticks = MAX_LINEAR_SLIDE_POSITION;
        }
        else if (ticks < MIN_LINEAR_SLIDE_POSITION) {
            ticks = MIN_LINEAR_SLIDE_POSITION;
        }
        linearSlideMotor.setTargetPosition(ticks);
        if (ticks > MIN_LINEAR_SLIDE_POSITION) {
            linearSlideMotor.setPower(LINEAR_SLIDE_POWER);
        }
        else {
            linearSlideMotor.setPower(LINEAR_SLIDE_POWER_DOWN);
        }
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}

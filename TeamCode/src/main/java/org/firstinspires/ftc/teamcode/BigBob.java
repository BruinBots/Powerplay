package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BigBob {


    // Class variables

    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;
    public DcMotorEx linearSlideMotor;


    public static int MAX_LINEAR_SLIDE_POSITON = 75;
    public static int MIN_LINEAR_SLIDE_POSITION = 0;

    public static double LINEAR_SLIDE_POWER = 0.2;

//    public DigitalChannel leftFrontSwitch;
//    public DigitalChannel rightFrontSwitch;

    //

    // constructor with map
    public BigBob(HardwareMap map) {
        // Drivetrain Motors
        DcMotorEx leftFrontMotor = map.get(DcMotorEx.class, "leftFrontMotor");
        DcMotorEx rightFrontMotor = map.get(DcMotorEx.class, "rightFrontMotor");
        DcMotorEx leftBackMotor = map.get(DcMotorEx.class, "leftBackMotor");
        DcMotorEx rightBackMotor = map.get(DcMotorEx.class, "rightBackMotor");

        DcMotorEx linearSlideMotor = map.get(DcMotorEx.class, "linearSlide");
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // Front Switches
//        leftFrontSwitch = map.get(DigitalChannel.class, "leftFrontSwitch");
//        rightFrontSwitch = map.get(DigitalChannel.class, "rightFrontSwitch");
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

    public void moveLinearSlide(int ticks) {
        linearSlideMotor.setTargetPosition(ticks);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideMotor.setPower(LINEAR_SLIDE_POWER);

    }
}

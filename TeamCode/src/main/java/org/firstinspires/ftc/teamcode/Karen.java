package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Karen  {


    // Class variables

    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;

    public DcMotorEx armMotor;
    public Servo clawServo;

    public DigitalChannel leftFrontSwitch;
    public DigitalChannel rightFrontSwitch;

    //

    // constructor with map
    public Karen (HardwareMap map) {
        // Drivetrain Motors
        DcMotorEx leftFrontMotor = map.get(DcMotorEx.class, "leftFrontMotor");
        DcMotorEx rightFrontMotor = map.get(DcMotorEx.class, "rightFrontMotor");
        DcMotorEx leftBackMotor = map.get(DcMotorEx.class, "leftBackMotor");
        DcMotorEx rightBackMotor = map.get(DcMotorEx.class, "rightBackMotor");

        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // arm assembly
        DcMotorEx armMotor = map.get(DcMotorEx.class, "armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Servo clawServo = map.get(Servo.class, "clawServo");


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
}

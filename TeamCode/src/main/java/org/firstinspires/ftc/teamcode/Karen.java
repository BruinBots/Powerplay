package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class Karen  {
    // Class variables

    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;

    public Encoder leftEncoder;
    public Encoder rightEncoder;
    public Encoder frontEncoder;

    public DcMotorEx armMotor;
    public Servo clawServo;

    public DigitalChannel leftFrontSwitch;
    public DigitalChannel rightFrontSwitch;

    public DigitalChannel leftOdoWheel;

    public static int MAX_ARM_POSITION = 160;
    public static int MIN_ARM_POSITION = 0;

    public static double CLAW_OPEN = 0.66;
    public static double CLAW_CLOSED = 0.04;

    public static double ARM_POWER = 0.8;

    public boolean leftSwitch;
    public boolean rightSwitch;

    //

    // constructor with map
    public Karen (HardwareMap map) {
        // Drivetrain Motors
        leftFrontMotor = map.get(DcMotorEx.class, "leftFrontMotor");
        rightFrontMotor = map.get(DcMotorEx.class, "rightFrontMotor");
        leftBackMotor = map.get(DcMotorEx.class, "leftBackMotor");
        rightBackMotor = map.get(DcMotorEx.class, "rightBackMotor");

        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        // Encoders
        leftEncoder = new Encoder(map.get(DcMotorEx.class, "leftFrontMotor"));
        rightEncoder = new Encoder(map.get(DcMotorEx.class, "rightFrontMotor"));
        leftEncoder.setDirection(Encoder.Direction.REVERSE); // might be wrong, but go builda reverses left by default so i reversed right, can check with op mode
        frontEncoder = new Encoder(map.get(DcMotorEx.class, "leftBackMotor"));

        // arm assembly
        armMotor = map.get(DcMotorEx.class, "armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        clawServo = map.get(Servo.class, "clawServo");

//        backOdoWheel = map.get(Encoder.class, );
//        leftOdoWheel = map.get(DigitalChannel.class, "leftDeadwheel");
//        rightOdoWheel =;

        //left odo wheel
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //right odo wheel
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // back odo wheel
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Front Switches
        leftFrontSwitch = map.get(DigitalChannel.class, "leftFrontSwitch");
        rightFrontSwitch = map.get(DigitalChannel.class, "rightFrontSwitch");
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

    public String center() {


        //true == switch activated
        leftSwitch = leftFrontSwitch.getState(); // reversed because of yes
        rightSwitch = rightFrontSwitch.getState(); // reversed because of yes

            if (leftSwitch && !rightSwitch) { // turn left
                this.moveBot(0, -0.2, 0, 1);
                return "turning left";
            } // right on
            else if (!leftSwitch && rightSwitch) { //turn right
                this.moveBot(0, 0.2, 0, 1);
                return "turning right";
            } // none on
            else if (!leftSwitch && !rightSwitch) { // do nothing
                this.moveBot(0.2, 0, 0, 1);
                return "moving forward!";
            } else if (leftSwitch && rightSwitch) { // break out
                return "found!";
            }

            return "error";
    }



    public void moveArm(int targetPos){
        armMotor.setTargetPosition(targetPos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_POWER);
    }

    public int getCurrentArmPos(){
        return armMotor.getCurrentPosition();
    }

    public void stop(){
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        armMotor.setPower(0);
    }
}

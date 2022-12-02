package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.analysis.integration.IterativeLegendreGaussIntegrator;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.EOCV.SleeveDetection;
import org.firstinspires.ftc.teamcode.util.Encoder;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

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

    public static int MAX_ARM_POSITION = 270;
    public static int MIN_ARM_POSITION = 0;

    public static double CLAW_OPEN = 0.5;
    public static double CLAW_CLOSED = 0.0;

    public static double ARM_POWER = 0.95;

    public boolean leftSwitch;
    public boolean rightSwitch;


    // For eocv

    public SleeveDetection sleeveDetection;
    public OpenCvCamera camera;

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
        //leftEncoder.setDirection(Encoder.Direction.REVERSE); // might be wrong, but go builda reverses left by default so i reversed right, can check with op mode
        frontEncoder = new Encoder(map.get(DcMotorEx.class, "leftBackMotor"));

        // arm assembly
        armMotor = map.get(DcMotorEx.class, "armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
                this.moveBot(0, -0.1, 0, 1);
                return "turning left";
            } // right on
            else if (!leftSwitch && rightSwitch) { //turn right
                this.moveBot(0, 0.1, 0, 1);
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

    public void openCam(HardwareMap map, Telemetry t){
        int cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection(t);
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1920,1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }


    public void stop(){
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        armMotor.setPower(0);
    }
}

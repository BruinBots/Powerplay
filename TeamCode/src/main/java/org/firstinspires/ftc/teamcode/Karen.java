package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.EOCV.SimTesterZone.AprilTagRecognitionPipeline;
import org.firstinspires.ftc.teamcode.EOCV.SimTesterZone.SleeveDetection;
import org.firstinspires.ftc.teamcode.util.Encoder;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Karen  {
    // Class variables

    public enum State {
        NORMAL,
        CENTERING,
        BACKING,
        DROPPING
    }

    public State currentState = State.NORMAL;

    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;

    public DcMotorEx slideMotor;

    public static final int MAX_LINEAR_SLIDE_POSITION = 1850;
    public static final int MIN_LINEAR_SLIDE_POSITION = 0;
    public static final int CONE_1 = 180;
    public static final int CONE_2 = 360;
    public static final int CONE_3 = 540;
    public static final int CONE_4 = 720;
    public static final int CONE_5 = 900;



   // public static final int MEDIUM_LINEAR_SLIDE_POSITION = 1450;
    public static final int LOW_LINEAR_SLIDE_POSITION = 900;

    public static final double LINEAR_SLIDE_POWER = 0.8;
    public static final double LINEAR_SLIDE_POWER_DOWN = 0.065;

    public Encoder leftEncoder;
    public Encoder rightEncoder;
    public Encoder frontEncoder;

    public DcMotorEx armMotor;
    public Servo clawServo;

    public DigitalChannel leftFrontSwitch;
    public DigitalChannel rightFrontSwitch;

    public DigitalChannel leftOdoWheel;

    // this is assuming that the arm starts up, and that position is set to 0 at the start
    public static int MAX_ARM_POSITION = 20;
    public static int MIN_ARM_POSITION = -310;

    public static double CLAW_OPEN = 0.5;
    public static double CLAW_CLOSED = 0.65;

    public static double ARM_POWER = 0.95;

    public boolean leftSwitch;
    public boolean rightSwitch;

    public DigitalChannel slideHardStop;
    public DistanceSensor frontDistanceSensor;
    public double pickupFromStack;

//    public double avgOverTen = 0;
//    public double [] avgVals = new double[10];


    // For eocv

    public SleeveDetection sleeveDetection;
    public AprilTagRecognitionPipeline pipeline;
    public OpenCvCamera camera;

    //

    // constructor with map
    public Karen (HardwareMap map) {
        slideHardStop = map.get(DigitalChannel.class, "slideBottomButton");
        // Drivetrain Motors
        leftFrontMotor = map.get(DcMotorEx.class, "leftFrontMotor");
        rightFrontMotor = map.get(DcMotorEx.class, "rightFrontMotor");
        leftBackMotor = map.get(DcMotorEx.class, "leftBackMotor");
        rightBackMotor = map.get(DcMotorEx.class, "rightBackMotor");

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        // Encoders
        leftEncoder = new Encoder(map.get(DcMotorEx.class, "leftFrontMotor"));
        rightEncoder = new Encoder(map.get(DcMotorEx.class, "rightFrontMotor"));
        //leftEncoder.setDirection(Encoder.Direction.REVERSE); // might be wrong, but go builda reverses left by default so i reversed right, can check with op mode
        frontEncoder = new Encoder(map.get(DcMotorEx.class, "leftBackMotor"));



        // arm assembly --OLD--
//        armMotor = map.get(DcMotorEx.class, "armMotor");
//        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        clawServo = map.get(Servo.class, "clawServo");

        slideMotor = map.get(DcMotorEx.class, "linearSlideMotor");
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        // leftFrontSwitch = map.get(DigitalChannel.class, "leftFrontSwitch");
        // rightFrontSwitch = map.get(DigitalChannel.class, "rightFrontSwitch");
    }

    public Karen (HardwareMap map, boolean fromAuto) {
        slideHardStop = map.get(DigitalChannel.class, "slideBottomButton");
        slideHardStop.setMode(DigitalChannel.Mode.INPUT);
        // Drivetrain Motors
        leftFrontMotor = map.get(DcMotorEx.class, "leftFrontMotor");
        rightFrontMotor = map.get(DcMotorEx.class, "rightFrontMotor");
        leftBackMotor = map.get(DcMotorEx.class, "leftBackMotor");
        rightBackMotor = map.get(DcMotorEx.class, "rightBackMotor");

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        // Encoders
        leftEncoder = new Encoder(map.get(DcMotorEx.class, "leftFrontMotor"));
        rightEncoder = new Encoder(map.get(DcMotorEx.class, "rightFrontMotor"));
        //leftEncoder.setDirection(Encoder.Direction.REVERSE); // might be wrong, but go builda reverses left by default so i reversed right, can check with op mode
        frontEncoder = new Encoder(map.get(DcMotorEx.class, "leftBackMotor"));

        // arm assembly
//        armMotor = map.get(DcMotorEx.class, "armMotor");
//        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // not resetting the position because it is from auto
        //armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        clawServo = map.get(Servo.class, "clawServo");

        slideMotor = map.get(DcMotorEx.class, "linearSlideMotor");
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        // leftFrontSwitch = map.get(DigitalChannel.class, "leftFrontSwitch");
        // rightFrontSwitch = map.get(DigitalChannel.class, "rightFrontSwitch");
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

    // movebot command that runs for a certain amount of time
    public void moveBotTimer(double drive, double rotate, double strafe, int time, ElapsedTime runTime){
        while(((int)(runTime.time() * 1000)) < time){
            this.moveBot(drive, rotate, strafe, 1);
        }
    }

    public int center(ElapsedTime runTime) {
        runTime.reset();
        int output = 0;

        //true == switch activated
        leftSwitch = leftFrontSwitch.getState(); // reversed because of yes
        rightSwitch = rightFrontSwitch.getState(); // reversed because of yes
            if (leftSwitch && !rightSwitch) { // turn left
                this.moveBotTimer(-0.1, 0, -0.1, 50, runTime);
            } // right on
            else if (!leftSwitch && rightSwitch) { //turn right
                this.moveBotTimer(-0.1, 0, 0.1, 50, runTime);
            } // none on
            else if (!leftSwitch && !rightSwitch) { // do nothing
                this.moveBot(0.2, 0, 0, 1);
                return 0;
            } else if (leftSwitch && rightSwitch) { // break out
                return 1;
            }
            return -1;
    }







    public void moveArm(int targetPos){
        armMotor.setTargetPosition(targetPos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_POWER);
    }

    public void moveLinearSlide(int ticks) {
        if (ticks > MAX_LINEAR_SLIDE_POSITION) {
            ticks = MAX_LINEAR_SLIDE_POSITION;
        }
        else if (ticks < MIN_LINEAR_SLIDE_POSITION) {
            ticks = MIN_LINEAR_SLIDE_POSITION;
        }
        slideMotor.setTargetPosition(ticks);
        if (ticks > MIN_LINEAR_SLIDE_POSITION) {
            slideMotor.setPower(LINEAR_SLIDE_POWER);
        }
        else {
            slideMotor.setPower(LINEAR_SLIDE_POWER_DOWN);
        }
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getCurrentArmPos(){
        return armMotor.getCurrentPosition();
    }

    public double ticksPerInch = 537.6 / 12.56; // TICKS PER REV / CIRCUMFERENCE

    public void openClaw(){
        clawServo.setPosition(CLAW_OPEN);
    }

    public void closeClaw() {
        clawServo.setPosition(CLAW_OPEN);
    }



    // c920 at 800 x 448 camera intrinsics, might need to be recalibrated
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.037; //  in meters

    public void openCam(HardwareMap map, Telemetry t){
        int cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//      sleeveDetection = new SleeveDetection(t);
        pipeline = new AprilTagRecognitionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
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

        slideMotor.setPower(0);
    }

    public boolean getSlideButton(){
        return slideHardStop.getState();
    }

    public int getSlidePos(){
        return slideMotor.getCurrentPosition();
    }

    public void resetSlideMotor(){
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    double e = Math.exp(1);
    double l = 1.04; // maximum
    double k = -8; // steepness
    double x0 = 0.6; // center
    double logMin = 0.1;
    double logMax = 0.9;
    double m1; // lower flatzone
    double m2; // higher flatzone

    public double logisticCurve(double x){
        // converts input to positive to be used with logistic curve
        double absX = Math.abs(x);
        double answer = 0;

        if (x < 0.1 || x > 0.95) {
            answer = l / (1.0 + Math.pow(e, -k * (absX - x0)));

            if(answer <= 0.025){
                answer = 0;
            } else if(answer >= 0.975){
                answer = 1;
            }
        } else {
            answer = x;
        }


        // clipping values of answer to ensure doesn't exceed motor limits


        // returns a negative value if input was negative
        return Math.copySign(answer, x);
    }

    public double flattenedLogisticCurve(double x){
        // converts input to positive to be used with logistic curve
        double absX = Math.abs(x);
        double answer = 0;


        // piecewise function
        if(absX <= logMin){
            m1 = this.logisticCurve(logMin) / logMin;
            answer = m1 * absX;
        } else if(absX <= logMax) {
            answer = this.logisticCurve(absX);
        } else {
            m2 = (1 - this.logisticCurve(logMax)) / (1 - logMax);
            answer = m2 * (absX - 1) + 1; //point-slop form
        }

        // clipping values of answer to ensure doesn't exceed motor limits
        if(answer > 1){
            answer = 1;
        }

        // returns a negative value if input was negative
        return Math.copySign(answer, x);
    }

    public void moveArmToLevel(int level){
        // level 0 =  ground
        // level 1 = ground cone
        // level 2 = second cone
        // level 3 = third
        // level 4 = fourth
        // level 5  = fifth
        // level 6 is top of pole

        if(level == 0){
            this.moveArm(MIN_ARM_POSITION);
        } else if (level == 1){
            this.moveArm(CONE_1);
        } else if (level == 2){
            this.moveArm(CONE_2);
        } else if (level == 3){
            this.moveArm(CONE_3);
        } else if (level == 4){
            this.moveArm(CONE_4);
        } else if (level == 5){
            this.moveArm(CONE_5);
        } else if (level == 6){
            this.moveArm(MAX_ARM_POSITION);
        }
    }

//    public void moveToConeStack(){
//        double rawDistance = frontDistanceSensor.getDistance();
//        while (rawDistance > pickupFromStack){
//            this.moveBot(0.2, 0, 0, 0);
//        }
//    }



}

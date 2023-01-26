package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.EOCV.SimTesterZone.AprilTagRecognitionPipeline;
import org.firstinspires.ftc.teamcode.EOCV.SimTesterZone.SleeveDetection;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Config
public class Karen  {
    // Class variables

    public enum State {
        NORMAL,
        CENTERING,
        BACKING,
        DROPPING
    }

    public static final double SLOW_SPEED = 0.35;
    public static final double FAST_SPEED = 0.85;
    public State currentState = State.NORMAL;

    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;

    public DcMotorEx slideMotor;

    public int targetSlidePos;

    public static final int MAX_LINEAR_SLIDE_POSITION = 1840;
    public static final int MIN_LINEAR_SLIDE_POSITION = 0;
    public static final int CONE_1 = MIN_LINEAR_SLIDE_POSITION; // not used
    public static final int CONE_2 = 100;
    public static final int CONE_3 = 290;
    public static final int CONE_4 = 500;
    public static final int CONE_5 = 650;

    public ModernRoboticsI2cColorSensor colorSensor;

    public static final double LINEAR_SLIDE_POWER = 0.96;
    public static final double LINEAR_SLIDE_POWER_DOWN = 0.065;

    public Encoder leftEncoder;
    public Encoder rightEncoder;
    public Encoder frontEncoder;

    public DcMotorEx armMotor;
    public Servo clawServo;

    public DigitalChannel leftFrontSwitch;
    public DigitalChannel rightFrontSwitch;

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
    DistanceUnit units_mm = DistanceUnit.MM;
    // in mm
    public double pickupFromStack = 130;

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


        // Color Sensor Setup
        colorSensor = map.get(ModernRoboticsI2cColorSensor.class,"colorSensor");
        colorSensor.enableLight(true);

        clawServo = map.get(Servo.class, "clawServo");

        slideMotor = map.get(DcMotorEx.class, "linearSlideMotor");
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setVelocityPIDFCoefficients(10, 2, 3,0);

        //left odo wheel
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //right odo wheel
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // back odo wheel
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontDistanceSensor = map.get(DistanceSensor.class, "frontDistanceSensor");
    }

    //in mm
    public double getFrontDistance(){
        return frontDistanceSensor.getDistance(units_mm);
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

        clawServo = map.get(Servo.class, "clawServo");

        slideMotor = map.get(DcMotorEx.class, "linearSlideMotor");
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //left odo wheel
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //right odo wheel
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // back odo wheel
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            targetSlidePos = MAX_LINEAR_SLIDE_POSITION;
        }
        else if (ticks < MIN_LINEAR_SLIDE_POSITION) {
            ticks = MIN_LINEAR_SLIDE_POSITION;
            targetSlidePos = MIN_LINEAR_SLIDE_POSITION;
        }
        slideMotor.setTargetPosition(ticks);
//        if (ticks > MIN_LINEAR_SLIDE_POSITION) {
//            slideMotor.setPower(LINEAR_SLIDE_POWER);
//        }
//        else {
//            slideMotor.setPower(LINEAR_SLIDE_POWER_DOWN);
//        }
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(LINEAR_SLIDE_POWER);
    }

    public int getCurrentArmPos(){
        return armMotor.getCurrentPosition();
    }

    public double ticksPerInch = 537.6 / 12.56; // TICKS PER REV / CIRCUMFERENCE

    public void openClaw(){
        clawServo.setPosition(CLAW_OPEN);
    }

    public void closeClaw() {
        clawServo.setPosition(CLAW_CLOSED);
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
        pipeline = new AprilTagRecognitionPipeline(tagsize, fx, fy, cx, cy, t);
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

    public static double e = Math.exp(1);
    public static double l = 1; // maximum
    public static double k = 1; // steepness
    public static double x0 = 0.3; // center
    public static double logMin = 0.1;
    public static double logMax = 0.9;
    public static double m1; // lower flatzone
    public static double m2; // higher flatzone
    public static double rampup = 0.25; // in seconds, when the speed starts to woo

    public double logisticCurve(double x){
        // converts input to positive to be used with logistic curve
        double absX = Math.abs(x);
        double answer = 0;


        if(x != 0) {
            answer = l / (1.0 + Math.pow(e, -k * (absX - x0)));
        }
        // returns a negative value if input was negative
        return Math.copySign(answer, x);
    }

    public static double convertToNewRange(double value, double OriginalMin, double OriginalMax, double NewMin, double NewMax){
        double newValue = (((value - OriginalMin) * (NewMax - NewMin)) / (OriginalMax - OriginalMin)) + NewMin;
        return newValue;
    }

    public double timeLogisticCurve(double time, double controller){
        // converts input to positive to be used with logistic curve
        double answer = 0;

        double x = convertToNewRange(time, 0, 1.5, -5, 5);
        if(time != 0) {
            answer = ((l - rampup) / (1.0 + Math.pow(e, -k * (x - x0)))) + rampup;
        }

        return Math.copySign(answer, controller);

        // returns a negative value if input was negative
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

    public void moveSlideToLevel(int level){
        // level 0 =  ground
        // level 1 = ground cone
        // level 2 = second cone
        // level 3 = third
        // level 4 = fourth
        // level 5  = fifth
        // level 6 is top of pole

        if(level == 1){
            this.moveLinearSlide(MIN_LINEAR_SLIDE_POSITION);
            targetSlidePos = MIN_LINEAR_SLIDE_POSITION;// also cone 1
        } else if (level == 2){
            this.moveLinearSlide(CONE_2);
            targetSlidePos = CONE_2;
        } else if (level == 3){
            this.moveLinearSlide(CONE_3);
            targetSlidePos = CONE_3;
        } else if (level == 4){
            this.moveLinearSlide(CONE_4);
            targetSlidePos = CONE_4;
        } else if (level == 5){
            this.moveLinearSlide(CONE_5);
            targetSlidePos = CONE_5;
        } else if (level == 6){
            this.moveLinearSlide(MAX_LINEAR_SLIDE_POSITION);
            targetSlidePos = MAX_LINEAR_SLIDE_POSITION;
        }
    }

    public void setSlideToLevel(int level){
        // level 0 =  ground
        // level 1 = ground cone
        // level 2 = second cone
        // level 3 = third
        // level 4 = fourth
        // level 5  = fifth
        // level 6 is top of pole

        if(level == 1){
            targetSlidePos = MIN_LINEAR_SLIDE_POSITION;// also cone 1
        } else if (level == 2){
            targetSlidePos = CONE_2;
        } else if (level == 3){
            targetSlidePos = CONE_3;
        } else if (level == 4){
            targetSlidePos = CONE_4;
        } else if (level == 5){
            targetSlidePos = CONE_5;
        } else if (level == 6){
            targetSlidePos = MAX_LINEAR_SLIDE_POSITION;
        }
    }

    public void moveToConeStack(){
        double rawDistance = frontDistanceSensor.getDistance(units_mm);
        if(rawDistance >= pickupFromStack){
            this.moveBot(0.3, 0, 0, 1);
        }
    }



}

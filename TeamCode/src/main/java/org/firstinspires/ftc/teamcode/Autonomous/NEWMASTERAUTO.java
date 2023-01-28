package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.EOCV.SimTesterZone.AprilTagRecognitionPipeline;
import org.firstinspires.ftc.teamcode.EOCV.SimTesterZone.BananaDetector;
import org.firstinspires.ftc.teamcode.Karen;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(group = "measure cut twice once")
public class NEWMASTERAUTO extends LinearOpMode {

    SampleMecanumDrive drive;
    Karen bot;


    // forward is positive x, left is positive y

    public void forward(double inches){
        Trajectory temp = drive.trajectoryBuilder(new Pose2d())
                .forward(inches) // move forward
                .build();

        drive.followTrajectory(temp);
    }


    // righr is postive and left is negative
    public void strafe(double inches){
        Trajectory temp = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(inches) // move forward
                .build();

        drive.followTrajectory(temp);
    }


    // RIGHT IS DEFAULT START SIDE, CLICK X FOR LEFT, B FOR RIGHT
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        bot = new Karen(hardwareMap);
        bot.openCam(hardwareMap, telemetry);

        // SETTING START POSE, HAVE TO CHANGE THIS LATER TO WHATEVER OUR FIELD POSITION IS
        Pose2d  startPose = new Pose2d(0,0, 0);
        drive.setPoseEstimate(startPose);

        // right is default start side
        boolean isLeft = false;
        int multiplier = 1;

        Trajectory moveback = drive.trajectoryBuilder(new Pose2d()) //  reseting its pose to 0
                .back(24)
                .build();

        // strafing left towards cone stack
        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d()) //  reseting its pose to 0
                .addDisplacementMarker(1, () -> {
                    // Runs 1 inch into trajectory
                    bot.moveSlideToLevel(5); //  move to constack level one to so we are ready to pick up
                })
                .lineToConstantHeading(new Vector2d(0, 40)) //  strafe left towards cone stack
                .build();


        //forward movement
        Trajectory straight = drive.trajectoryBuilder(new Pose2d())
                .forward(26) // move forward
                .build();

        // right movement if red
        Trajectory strafeRight = drive.trajectoryBuilder(straight.end())
                .strafeRight(24) // move right
                .build();

        Trajectory strafeLeft = drive.trajectoryBuilder(straight.end())
                .strafeLeft(24) // move left
                .build();


        while(!isStarted()){
            if(gamepad1.x){
                isLeft = true;
            }

            if(gamepad1.b){
                isLeft = false;
            }

            if(isLeft){
                multiplier = -1;
            }

            telemetry.addData("Parking: ", bot.pipeline.getParkingPosition());
            telemetry.addData("Blue: ", isLeft);
            telemetry.update();
        }

        // init banana detector
        BananaDetector bananaVision = new BananaDetector(telemetry);
        waitForStart(); // AUTO STARTS AFTER PLAY IS CLICKED -------------

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(1, () -> {
                    // Runs 1 inch into trajectory
                    bot.moveSlideToLevel(6);
                })
                .waitSeconds(1)
                .forward(2)
                .strafeLeft(12.5 * multiplier)
                .forward(7)
                .addDisplacementMarker(() -> {
                    // rus after forward movememt
                    bot.openClaw();
                })
                .waitSeconds(.5)
                .back(8)
                .waitSeconds(.5)
                .strafeLeft(12 * multiplier)
                .waitSeconds(.5)
                .forward(54)
                .waitSeconds(.5)
                .addDisplacementMarker(() -> {
                    // rus after forward movememt
                    bot.moveSlideToLevel(5);
                })
                .turn(Math.toRadians(-90 * multiplier))
                .waitSeconds(.5)
                .forward(40)
                .waitSeconds(.5)
                .build();

        TrajectorySequence moveback1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(24)
                .waitSeconds(1)
                .turn(Math.toRadians(-45))
                .waitSeconds(1)
                .forward(6)
                .build();


        // close claws and grab position
        bot.closeClaw();
        AprilTagRecognitionPipeline.ParkingPosition parkZone = bot.pipeline.getParkingPosition(); // default go center
        sleep(1000);


        // ends in front of cone stack
        drive.followTrajectorySequence(trajSeq1);

        double targetAngle = bot.getHeading();
        while((bot.colorSensor.blue() < (bot.colorSensor.red() * 3)) & !isStopRequested()){
            bot.gyroStrafe(0.2, targetAngle);
        }

        bot.moveBot(0, 0, 0, 1);

        sleep(1000);
        while(bot.getFrontDistance() >= bot.pickupFromStack && !isStopRequested()) {
            bot.moveToConeStack();
        }

        sleep(1000);
        bot.closeClaw();
        sleep(1000);
        bot.moveSlideToLevel(6);
        // sleep(1000);

         drive.followTrajectorySequence(moveback1);

//        while(bot.getFrontDistance() <= 312 && !isStopRequested()){
//            bot.moveBot(-0.2, 0, 0, 0);
//        }

        sleep(1000);
        bot.openClaw();

        Trajectory back = drive.trajectoryBuilder(new Pose2d())
                .back(10) // move forward
                .build();

        sleep(1000);




//        drive.followTrajectory(traj1);
//        sleep(2000);
//        drive.followTrajectory(moveback);
//        drive.followTrajectory(traj2);
//        sleep(2000);
//        drive.followTrajectory(traj3);
        //drive straight first, then do corrections


//        telemetry.addData("Parking: ", parkZone);
//       drive.followTrajectory(straight);
///
//        if(parkZone == AprilTagRecognitionPipeline.ParkingPosition.LEFT){
//            drive.followTrajectory(strafeLeft);
//        } else if (parkZone == AprilTagRecognitionPipeline.ParkingPosition.RIGHT){
//            drive.followTrajectory(strafeRight);
//        }



    }
}
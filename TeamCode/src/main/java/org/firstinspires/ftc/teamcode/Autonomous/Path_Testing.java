package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.EOCV.SimTesterZone.AprilTagRecognitionPipeline;
import org.firstinspires.ftc.teamcode.Karen;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Vector;

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
public class Path_Testing extends LinearOpMode {

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


    // code is written for red and swapped for blue if needed
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        bot = new Karen(hardwareMap);


        // SETTING START POSE, HAVE TO CHANGE THIS LATER TO WHATEVER OUR FIELD POSITION IS
        Pose2d  startPose = new Pose2d(0,0, 0);
        drive.setPoseEstimate(startPose);


        bot.openCam(hardwareMap, telemetry);

        // code is written for red and swapped for blue if needed
        boolean isBlue = false;
        int multiplier = 0;


        // move left and move forward and also raise arm at the same time
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(1, () -> {
                    // Runs 1 inch into trajectory
                    bot.moveSlideToLevel(6);
                })
                .splineToConstantHeading(new Vector2d(2, 0), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(2, 12), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(10, 12), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    // rus after forward movememt
                    bot.openClaw();
                })
                .build();

        Trajectory test = drive.trajectoryBuilder(startPose)
//                .addDisplacementMarker(1, () -> {
//                    // Runs 1 inch into trajectory
//                    bot.moveSlideToLevel(6);
//                })
                .splineToConstantHeading(new Vector2d(12, 0), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(0))
                //.splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(0))
//                .addDisplacementMarker(() -> {
//                    // rus after forward movememt
//                    bot.openClaw();
//                })
                .build();



//        Trajectory wait = drive.trajectoryBuilder(traj1.end())
//                .addTemporalMarker(5, () -> {}) // wait 5 seconds cuhz //  strafe right, then park
//                .build();


        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(1, () -> {
                    // Runs 1 inch into trajectory
                    bot.moveSlideToLevel(6);
                })
                .waitSeconds(1)
                .forward(2)
                .strafeLeft(12.5)
                .forward(8)
                .addDisplacementMarker(() -> {
                    // rus after forward movememt
                    bot.openClaw();
                })
                .waitSeconds(1)
                .back(8)
                .waitSeconds(1)
                .strafeLeft(12.5)
                .waitSeconds(1)
                .forward(48)
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    // rus after forward movememt
                    bot.moveSlideToLevel(5);
                })
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .forward(50)
                .waitSeconds(1)
                .build();

        // going to movebck, turn right, and strafe to parallel with alliance, then next is strafe to cone stack
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d()) //  reseting its pose to 0
                .lineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90))) //  strafe right, then park
                .build();

        Trajectory moveback = drive.trajectoryBuilder(new Pose2d()) //  reseting its pose to 0
                .back(4)
                .build();

        // strafing left towards cone stack
        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d()) //  reseting its pose to 0
                .addDisplacementMarker(1, () -> {
                    // Runs 1 inch into trajectory
                    bot.moveSlideToLevel(5); //  move to constack level one to so we are ready to pick up
                })
                .lineToConstantHeading(new Vector2d(0, 40)) //  strafe left towards cone stack
                .build();


        // self cetnering, here, do i have to over shoot or undershoot the cone stack
        //selfcenter();




        //forward movement
        Trajectory straight = drive.trajectoryBuilder(traj2.end())
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
                isBlue = true;
            }

            if(gamepad1.b){
                isBlue = false;
            }

            if(isBlue){
                multiplier = -1;
            }


            telemetry.addData("Parking: ", bot.pipeline.getParkingPosition());
            telemetry.addData("Blue: ", isBlue);
            telemetry.update();
        }

        waitForStart(); // AUTO STARTS AFTER PLAY IS CLICKED -------------
        bot.closeClaw();

        AprilTagRecognitionPipeline.ParkingPosition parkZone = bot.pipeline.getParkingPosition(); // default go right

        drive.followTrajectorySequence(trajSeq1);
        bot.closeClaw();
        sleep(1000);
        bot.moveSlideToLevel(6);
        while(bot.slideMotor.isBusy()){

        }

        Trajectory back = drive.trajectoryBuilder(traj2.end())
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
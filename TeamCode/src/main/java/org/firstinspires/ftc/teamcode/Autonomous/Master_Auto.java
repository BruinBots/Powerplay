package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.EOCV.SimTesterZone.SleeveDetection;
import org.firstinspires.ftc.teamcode.Karen;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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
public class Master_Auto extends LinearOpMode {

    SampleMecanumDrive drive;
    Karen bot;

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
        bot.openCam(hardwareMap, telemetry);

        // code is written for red and swapped for blue if needed
        boolean isBlue = false;
        int multiplier = 0;


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
                isBlue = true;
            }

            if(gamepad1.b){
                isBlue = false;
            }

            if(isBlue){
                multiplier = -1;
            }

            telemetry.addData("AvgColor: ", bot.sleeveDetection.avgColorVal);
            telemetry.addData("Parking: ", bot.sleeveDetection.getPosition());
            telemetry.addData("Blue: ", isBlue);
            telemetry.update();
        }

        waitForStart();

        // AUTO DROP CODE ----


        // AUTO DROP CODE -----



        //drive straight first, then do corrections

        SleeveDetection.ParkingPosition parkZone = bot.sleeveDetection.getPosition(); // default go right
        telemetry.addData("Parking: ", parkZone);
        drive.followTrajectory(straight);

        if(parkZone == SleeveDetection.ParkingPosition.LEFT){
            drive.followTrajectory(strafeLeft);
        } else if (parkZone == SleeveDetection.ParkingPosition.RIGHT){
            drive.followTrajectory(strafeRight);
        }

    }
}
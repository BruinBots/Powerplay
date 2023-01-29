/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Config
@TeleOp(name="FROM_AUTO_TwoController", group="Iterative Opmode")
public class $FromAutoTwoControllerMecanumOpMode extends OpMode {

    // Declare OpMode members.

    double drive = 0.0;
    double turn = 0.0;
    double strafe = 0.0;

    //bot.targetSlidePos = 0;
    int slidePos = 1;
    int slideStep = 1;

    double armPower = 0.0;
    double clawPos = 0.0;

    boolean lastValofX = false;
    String seekStatus = "";
    boolean dropped = false;
    boolean found = false;
    boolean isLastValofA = false;
    boolean switchToBacking = false;

    boolean lastValOfA = false;
    boolean lastValOfY = false;

    ElapsedTime runTime = new ElapsedTime();

    int armPos;
    int centerTemp = 0;

    Karen bot;

    SampleMecanumDrive roadRunnerDrive;

    Trajectory correctAfterCetner;

    int initPos;

    //
    @Override
    public void init() {
        bot = new Karen(hardwareMap, true);
        telemetry.addData("Status", "Initialized");
       // armPos = bot.armMotor.getCurrentPosition();

        roadRunnerDrive = new SampleMecanumDrive(hardwareMap);

        correctAfterCetner = roadRunnerDrive.trajectoryBuilder(new Pose2d())
                .back(2.5)
                .build();

        bot.slideMotor.setVelocityPIDFCoefficients(10, 2, 3,0);
        initPos = bot.frontEncoder.getCurrentPosition();
    }

    //
    @Override
    public void init_loop() {
        //armPos = bot.armMotor.getCurrentPosition();
        telemetry.addData("Init: ", initPos);
        telemetry.addData("Current: ", bot.frontEncoder.getCurrentPosition());
        telemetry.addData("Divsion ", (bot.frontEncoder.getCurrentPosition() - initPos) / 1304);
        telemetry.addData("Inches Moved: ", bot.encoderToInches(initPos, bot.frontEncoder.getCurrentPosition()));
        telemetry.addData("Angle: ", bot.getHeading());
        telemetry.addData("Pid: ",bot.slideMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
    }

    //
    @Override
    public void start() {
        runTime.reset();
    }

    //

    @Override
    public void loop() {
        telemetry.addData("Front Dist: ", bot.getFrontDistance());
        drive = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

//        turn = Math.copySign(Math.pow(turn, 2), turn);
//        strafe = Math.copySign(Math.pow(strafe, 2), strafe);
//        drive = Math.copySign(Math.pow(drive, 2), drive);

        telemetry.addData("LEFT_x: ", drive);
        telemetry.addData("LEFT_y: ", strafe);
        telemetry.addData("RIGHT_x: ", turn);

         // possible logistic curve implementation
        drive = bot.logisticCurve(drive);
        strafe = bot.logisticCurve(strafe);
        turn = bot.logisticCurve(turn);

        if(gamepad1.right_bumper){
            bot.moveBot(drive, turn, strafe, bot.FAST_SPEED);
        } else if(gamepad1.x){
            bot.moveToConeStack();
        } else {
            bot.moveBot(drive, turn, strafe, bot.SLOW_SPEED);
        }

        telemetry.addData("Drive: ", drive);
        telemetry.addData("strafe: ", strafe);
        telemetry.addData("turn: ", turn);

        telemetry.addData("Red: ", bot.colorSensor.red());
        telemetry.addData("Green: ", bot.colorSensor.green());
        telemetry.addData("Blue: ", bot.colorSensor.blue());
        telemetry.addData("Color: ", bot.colorSensor.getNormalizedColors());

        // Open and close
        if (gamepad2.left_bumper) {
            bot.closeClaw();
        }
        if (gamepad2.right_bumper) {
            bot.openClaw();
        }



        // moveing slide with gamepad2 right joystick
        bot.targetSlidePos += -gamepad2.right_stick_y * 65; // second number is slide step

        // dpad up and down steps between levels
        // dpad left and right goes from level 1 to 6
        if(gamepad2.y && !lastValOfY){
            if(slideStep < 6) {
                slideStep += 1;
            }
            bot.setSlideToLevel(slideStep);
        } else if(gamepad2.a && !lastValOfA){
            if(slideStep > 1){
                slideStep -= 1;
            }
            bot.setSlideToLevel(slideStep);
        } else if(gamepad2.dpad_up){
            slideStep = 6;
            bot.setSlideToLevel(slideStep);
        } else if(gamepad2.dpad_down){
            slideStep = 1;
            bot.setSlideToLevel(slideStep);
        }

        lastValOfA = gamepad2.a;
        lastValOfY = gamepad2.y;


        if(!bot.getSlideButton() && (-gamepad2.right_stick_y < 0)){
            // zeroes out the slide if the button is clicked
            bot.resetSlideMotor();
            bot.slideMotor.setPower(0);
            bot.targetSlidePos = 0;
        } else {
            bot.moveLinearSlide(bot.targetSlidePos);
        }

        telemetry.addData("slideStep: ", slideStep);
        telemetry.addData("codeSlidePos: ", bot.targetSlidePos);
        telemetry.addData("Actual SlidePOs: ", bot.getSlidePos());
        telemetry.addData("HardStop: ", bot.getSlideButton());

        telemetry.addData("leftFront: ", bot.leftFrontMotor.getPower());
        telemetry.addData("rightFront: ", bot.rightFrontMotor.getPower());
        telemetry.addData("leftback: ", bot.leftBackMotor.getPower());
        telemetry.addData("rightBack: ", bot.rightBackMotor.getPower());



        telemetry.addData("Time: ", runTime.time());
        telemetry.addData("Target Servo: ", clawPos);
        telemetry.addData("acc Claw Pos: ", bot.clawServo.getPosition());

        telemetry.addData("HardStop: ", bot.getSlideButton());

        telemetry.addData("leftOdo: ", bot.leftEncoder.getCurrentPosition());
        telemetry.addData("rightOdo: ", bot.rightEncoder.getCurrentPosition());
        telemetry.addData("backOdo: ", bot.frontEncoder.getCurrentPosition());
    }


    @Override
    public void stop() {
        bot.stop();
    }

}






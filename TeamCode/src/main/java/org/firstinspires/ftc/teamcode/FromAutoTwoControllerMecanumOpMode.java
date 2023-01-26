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
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Config
@TeleOp(name="FROM_AUTO_TwoController", group="Iterative Opmode")
public class FromAutoTwoControllerMecanumOpMode extends OpMode {

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

    boolean lastValOfUp2 = false;
    boolean lastValOfDown2 = false;

    ElapsedTime runTime = new ElapsedTime();

    int armPos;
    int centerTemp = 0;

    Karen bot;

    SampleMecanumDrive roadRunnerDrive;

    Trajectory correctAfterCetner;

    //
    @Override
    public void init() {


        bot = new Karen(hardwareMap);
        telemetry.addData("Status", "Initialized");
       // armPos = bot.armMotor.getCurrentPosition();

        roadRunnerDrive = new SampleMecanumDrive(hardwareMap);

        correctAfterCetner = roadRunnerDrive.trajectoryBuilder(new Pose2d())
                .back(2.5)
                .build();


//        int motorIndex = ((bot.slideMotor).getPortNumber());
//        DcMotorControllerEx motorController = (DcMotorControllerEx)bot.slideMotor.getController();
//        PIDFCoefficients pidNew = new PIDFCoefficients(10, 1, 3, 0);
//        motorController.setPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_TO_POSITION, pidNew);
//        bot.slideMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);
        // Identify motor port for slideMotor

//        bot.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        int motorIndex = ((bot.slideMotor).getPortNumber());
//        DcMotorControllerEx motorControllerEx = (DcMotorControllerEx) bot.slideMotor.getController();
//
//        // Read out existing PIDF values
//        PIDFCoefficients pidModified = motorControllerEx.getPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_TO_POSITION);
//        telemetry.addData("PIDModified: ", pidModified);
//
//        // change coefficients using methods included with DcMotorEx class.
//        PIDFCoefficients pidNew = new PIDFCoefficients(10, 2, 3,0);
//        motorControllerEx.setPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_TO_POSITION, pidNew);
//
//        bot.slideMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidModified);
//

        bot.slideMotor.setVelocityPIDFCoefficients(10, 2, 3,0);
    }

    //
    @Override
    public void init_loop() {
        //armPos = bot.armMotor.getCurrentPosition();
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

//        if(gamepad1.left_stick_y != 0 && gamepad1.left_stick_x != 0 && gamepad1.right_stick_x != 0){
//            runTime.reset();
//            drive = 0;
//            strafe = 0;
//            turn = 0;
//        } else {
//            double time = runTime.time();
//            if(gamepad1.left_stick_y != 0 ) {
//                drive = bot.timeLogisticCurve(time, drive);
//            }
//            if (gamepad1.left_stick_x != 0) {
//                strafe = bot.timeLogisticCurve(time, strafe);
//            }
//            if (gamepad1.right_stick_x != 0) {
//                turn = bot.timeLogisticCurve(time, turn);
//            }
//        }

        if(gamepad1.right_bumper){
            bot.moveBot(drive, turn, strafe, bot.FAST_SPEED);
        } else {
            bot.moveBot(drive, turn, strafe, bot.SLOW_SPEED);
        }

        telemetry.addData("Drive: ", drive);
        telemetry.addData("strafe: ", strafe);
        telemetry.addData("turn: ", turn);

//        telemetry.addData("gamepadx: ", gamepad1.left_stick_x);
//        telemetry.addData("gamepady: ", -gamepad1.left_stick_y);
//        telemetry.addData("gamepadr: ", gamepad1.right_stick_x);
//        telemetry.addData("Drive: ", drive);
//        telemetry.addData("strafe: ", strafe);
//        telemetry.addData("turn: ", turn);

        telemetry.addData("Red: ", bot.colorSensor.red());
        telemetry.addData("Green: ", bot.colorSensor.green());
        telemetry.addData("Blue: ", bot.colorSensor.blue());
        telemetry.addData("Color: ", bot.colorSensor.getNormalizedColors());
        // Open and close
        if (gamepad2.right_bumper) {
            bot.openClaw();
        }
        if (gamepad2.left_bumper) {
            bot.closeClaw();
        }

        //arm -----------------

        // assume arm starts all the way up
//        if (gamepad2.dpad_down) { // arm down
//            armPos -= 35; // positive due to motor rotation flipped
//            // Lowest arm can go for safety,
//            if (armPos < Karen.MIN_ARM_POSITION){ // 40
//                armPos = Karen.MIN_ARM_POSITION;
//            }
//
//            bot.moveArm(armPos);
//            armPos = bot.getCurrentArmPos();
//            telemetry.addData("arm down", "");
//        } else if (gamepad2.dpad_up) {
//            armPos += 35;
//
//            if(armPos > Karen.MAX_ARM_POSITION){ // -365
//                armPos = Karen.MAX_ARM_POSITION;
//            }
//
//            bot.moveArm(armPos);
//            armPos = bot.getCurrentArmPos();
//            telemetry.addData("arm up", "");
//        } else {
//            bot.moveArm(armPos);
//            telemetry.addData("arm still", "");
//        }

//        if (gamepad2.dpad_up) {
//            // bounds for encoder target
//            if (linearSlide > Karen.MAX_LINEAR_SLIDE_POSITION) {
//                linearSlide = Karen.MAX_LINEAR_SLIDE_POSITION;
//            }
//            else {
//                linearSlide += 22;
//            }
//        }
        ;

        bot.targetSlidePos += -gamepad2.right_stick_y * 65; // second number is slide step

//        if (bot.targetSlidePos > Karen.MAX_LINEAR_SLIDE_POSITION) {
//            bot.targetSlidePos = Karen.MAX_LINEAR_SLIDE_POSITION;
//        } else if (bot.targetSlidePos < Karen.MIN_LINEAR_SLIDE_POSITION) {
//            bot.targetSlidePos = Karen.MIN_LINEAR_SLIDE_POSITION;
//        }

        if(gamepad2.dpad_up && !lastValOfUp2){
            if(slideStep < 6) {
                slideStep += 1;
            }
            bot.setSlideToLevel(slideStep);
        } else if(gamepad2.dpad_down && !lastValOfDown2){
            if(slideStep > 1){
                slideStep -= 1;
            }
            bot.setSlideToLevel(slideStep);
        }

        lastValOfUp2 = gamepad2.dpad_up;
        lastValOfDown2 = gamepad2.dpad_down;


        if(!bot.getSlideButton() && (-gamepad2.right_stick_y < 0)){
            // zeroes out the slide if the button is clicked
            bot.resetSlideMotor();
        } else {
            bot.moveLinearSlide(bot.targetSlidePos);
        }

        telemetry.addData("slideStep: ", slideStep);
        telemetry.addData("codeSlidePos: ", bot.targetSlidePos);
        telemetry.addData("Actual SlidePOs: ", bot.getSlidePos());


        // zeroes out the slide if the button is clicked

        // move down unless button is clicked
//        else if (gamepad2.dpad_down && bot.getSlideButton()) {
//
//            //bounds for encoder target
//            if (linearSlide < Karen.MIN_LINEAR_SLIDE_POSITION) {
//                linearSlide = Karen.MIN_LINEAR_SLIDE_POSITION;
//            }
//            else {
//                linearSlide -= 22;
//            }
//        }

        //emergency override - how to implement idk

        // all the way down for collection
        if (gamepad2.a) {
            bot.targetSlidePos = Karen.MIN_LINEAR_SLIDE_POSITION;
        }
        // transit/ground junction, little bit off the ground to avoid contact
        else if (gamepad2.b) {
            bot.targetSlidePos = Karen.CONE_1;
        }
        // all the way up for low pole scoring
        else if (gamepad2.y) {
            bot.targetSlidePos = Karen.MAX_LINEAR_SLIDE_POSITION;
        }

        telemetry.addData("slide button: ", bot.getSlideButton());





        //hold to center
//        if(gamepad1.x && !dropped){
//            seekStatus = bot.center();
//            if(seekStatus.equals("found!") && !lastValofX){
//                roadRunnerDrive.followTrajectory(correctAfterCetner);
//                clawPos = Karen.CLAW_OPEN;
//                dropped = true;
//            }
//        }

//        if(gamepad1.x && !lastValofX) // last val since only exec once
//            bot.currentState = Karen.State.CENTERING;
//        else if (!gamepad1.x)
//            bot.currentState = Karen.State.NORMAL;
//
//        if(bot.currentState == Karen.State.CENTERING) {
//            switchToBacking = false;
//            centerTemp = bot.center(runTime);
//            if(centerTemp == 1) {
//                bot.currentState = Karen.State.BACKING;
//            }
//
//        }

//        if(bot.currentState == Karen.State.BACKING && !switchToBacking){
//            int temp = bot.leftEncoder.getCurrentPosition();
//            while ((Math.abs(bot.leftEncoder.getCurrentPosition() - temp) < 3500) && gamepad1.x){ // 2000 arbitrary, about quarter of wheel spin
//                bot.moveBot(-.15,0,0,1);
//            }
//
//            switchToBacking = true;
//            bot.currentState = Karen.State.DROPPING;
//        }
//
//        // dropp the thang!
//        if(bot.currentState == Karen.State.DROPPING){
//            clawPos = bot.CLAW_OPEN;
//            bot.currentState = Karen.State.NORMAL;
//        }

//        if (gamepad2.a && !isLastValofA){
//            roadRunnerDrive.followTrajectory(correctAfterCetner);
//            clawPos = bot.CLAW_OPEN;
//        }
//
//        isLastValofA = gamepad2.a;
//        telemetry.addData("Seek: ", seekStatus);
//        telemetry.addData("DROP: ", dropped);
//        lastValofX = gamepad1.x;


        //bot.clawServo.setPosition(clawPos);

        //telemetry.addData("Distance: ",bot.frontDistanceSensor.getDistance());

        telemetry.addData("Time: ", runTime.time());
//        telemetry.addData("STATE: ", bot.currentState);
        telemetry.addData("Target Servo: ", clawPos);
        telemetry.addData("acc Claw Pos: ", bot.clawServo.getPosition());
        // telemetry.addData("armPos:", bot.getCurrentArmPos());

        telemetry.addData("Actual Slide: ", bot.getSlidePos());
        telemetry.addData("Target Slide: ", bot.targetSlidePos);
        telemetry.addData("HardStop: ", bot.getSlideButton());

        telemetry.addData("leftOdo: ", bot.leftEncoder.getCurrentPosition());
        telemetry.addData("rightOdo: ", bot.rightEncoder.getCurrentPosition());
        telemetry.addData("backOdo: ", bot.frontEncoder.getCurrentPosition());
       // telemetry.addData("leftMotor", bot.leftFrontMotor.getPower());
//        telemetry.addData("leftEncoder", bot.leftEncoder.getCurrentPosition());
//        telemetry.addData("rightEncoder", bot.rightEncoder.getCurrentPosition());
//        telemetry.addData("frontEncoder", bot.frontEncoder.getCurrentPosition());
//        telemetry.addData("leftEncoderCorVel", bot.leftEncoder.getCorrectedVelocity());
//        telemetry.addData("rightEncoderCorVel", bot.rightEncoder.getCorrectedVelocity());
//        telemetry.addData("frontEncoderCorVel", bot.frontEncoder.getCorrectedVelocity());
//        telemetry.addData("leftEncoderRawVel", bot.leftEncoder.getRawVelocity());
//        telemetry.addData("rightEncoderRawVel", bot.rightEncoder.getRawVelocity());
//        telemetry.addData("frontEncoderRawVel", bot.frontEncoder.getRawVelocity());


    }


    @Override
    public void stop() {
        bot.stop();
    }

}






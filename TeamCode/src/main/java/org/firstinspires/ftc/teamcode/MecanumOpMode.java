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


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.BigBob.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.BigBob.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.BigBob.CLAW_ZERO_POSITION;
import static org.firstinspires.ftc.teamcode.BigBob.MAX_LINEAR_SLIDE_POSITON;
import static org.firstinspires.ftc.teamcode.BigBob.MIN_LINEAR_SLIDE_POSITION;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class MecanumOpMode extends OpMode
{
    // Declare OpMode members.

    double drive = 0.0;
    double turn = 0.0;
    double strafe = 0.0;

    int linearSlide = 0;
    double clawPos = CLAW_ZERO_POSITION;

    BigBob bot;

    //
    @Override
    public void init() {

        bot = new BigBob(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    //
    @Override
    public void init_loop() {
    }

    //
    @Override
    public void start() {
    }

    //
    @Override
    public void loop() {
        drive = -gamepad1.left_stick_y * -0.5;
        strafe = gamepad1.left_stick_x * -0.5;
        turn = gamepad1.right_stick_x * -0.5;

        bot.moveBot(drive, turn, strafe, 0.5);

        // Linear Slide Code`
        if (gamepad1.dpad_up) {
            linearSlide += 10;
        }
        else if (gamepad1.dpad_down) {
            linearSlide -= 10;
        }
        else if (gamepad1.y) {
            linearSlide = MAX_LINEAR_SLIDE_POSITON;
        }
        else if (gamepad1.a) {
            linearSlide = 0;
        }

        if (linearSlide > MAX_LINEAR_SLIDE_POSITON) {
            linearSlide = MAX_LINEAR_SLIDE_POSITON;
        }
        else if (linearSlide < MIN_LINEAR_SLIDE_POSITION) {
            linearSlide = MIN_LINEAR_SLIDE_POSITION;
        }

        if (gamepad1.dpad_right) { // open
            if (clawPos < CLAW_OPEN) {
                clawPos += 0.05;
            }
            if (clawPos > CLAW_OPEN) {
                clawPos = CLAW_OPEN;
            }
        }
        else if (gamepad1.dpad_left) { // close
            if (clawPos > CLAW_CLOSED) {
                clawPos -= 0.05;
            }
            if (clawPos < CLAW_CLOSED) {
                clawPos = CLAW_CLOSED;
            }
        }

        bot.clawServo.setPosition(clawPos);

        telemetry.addData("linearSlideMotor", linearSlide);
        telemetry.addData("linearSlideEncoder", bot.linearSlideMotor.getCurrentPosition());
        telemetry.addData("clawServoExpected", clawPos);
        telemetry.addData("clawServo", bot.clawServo.getPosition());
        telemetry.update();

        bot.moveLinearSlide(linearSlide);

        try {
            sleep(20);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //telemetry.addData("Left Switch", bot.leftFrontSwitch.getState());
       // telemetry.addData("Right Switch", bot.rightFrontSwitch.getState());
    }


    @Override
    public void stop() {
    }



}

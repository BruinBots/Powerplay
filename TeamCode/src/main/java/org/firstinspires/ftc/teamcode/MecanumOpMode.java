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

import static org.firstinspires.ftc.teamcode.BigBob.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.BigBob.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.BigBob.MAX_LINEAR_SLIDE_POSITION;
import static org.firstinspires.ftc.teamcode.BigBob.MIN_LINEAR_SLIDE_POSITION;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Basic: TeleOp", group="Iterative Opmode")
public class MecanumOpMode extends OpMode
{
    // Declare OpMode members.

    double drive = 0.0;
    double turn = 0.0;
    double strafe = 0.0;

    int linearSlide = 0;
    double clawPos = BigBob.CLAW_ZERO_POSITION;

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
            if (linearSlide > MAX_LINEAR_SLIDE_POSITION) {
                linearSlide = MAX_LINEAR_SLIDE_POSITION;
            }
            else {
                linearSlide += 10;
            }
        }
        else if (gamepad1.dpad_down) {
            if (linearSlide < MIN_LINEAR_SLIDE_POSITION) {
                linearSlide = MIN_LINEAR_SLIDE_POSITION;
            }
            else {
                linearSlide -= 10;
            }
        }
        else if (gamepad1.y) {
            linearSlide = BigBob.MAX_LINEAR_SLIDE_POSITION;
        }
        else if (gamepad1.x) {
            linearSlide = BigBob.MEDIUM_LINEAR_SLIDE_POSITION;
        }
        else if (gamepad1.b) {
            linearSlide = BigBob.LOW_LINEAR_SLIDE_POSITION;
        }
        else if (gamepad1.a) {
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
        else if (gamepad1.left_bumper) {
            clawPos = CLAW_OPEN;
        }
        else if (gamepad1.left_trigger > 0.2) {
            clawPos = CLAW_OPEN - (CLAW_OPEN - CLAW_CLOSED) * gamepad1.left_trigger;
        }
        else if (gamepad1.left_trigger > 0.8) {
            clawPos = CLAW_CLOSED;
        }

        telemetry.addData("linearSlideMotor", linearSlide);
        telemetry.addData("linearSlideEncoder", bot.linearSlideMotor.getCurrentPosition());
        telemetry.addData("clawServoExpected", clawPos);
        telemetry.addData("clawServo", bot.clawServo.getPosition());
        telemetry.update();

        bot.moveClaw(clawPos);
        bot.moveLinearSlide(linearSlide);

        try {
            sleep(20);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }





    }
    @Override
    public void stop() {
    }
}

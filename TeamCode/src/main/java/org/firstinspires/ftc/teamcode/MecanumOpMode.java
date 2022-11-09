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


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="charlie, pls click this op mode", group="Iterative Opmode")
public class MecanumOpMode extends OpMode {
    // Declare OpMode members.

    double drive = 0.0;
    double turn = 0.0;
    double strafe = 0.0;

    double armPower = 0.0;
    double clawPos = 0.0;

    int armPos;

    Karen bot;

    //
    @Override
    public void init() {

        bot = new Karen(hardwareMap);
        telemetry.addData("Status", "Initialized");
        armPos = bot.armMotor.getCurrentPosition();
    }

    //
    @Override
    public void init_loop() {
        armPos = bot.armMotor.getCurrentPosition();
    }

    //
    @Override
    public void start() {
    }

    //
    @Override
    public void loop() {
        drive = -gamepad1.left_stick_y * 0.5;
        strafe = gamepad1.left_stick_x * 0.5;
        turn = gamepad1.right_stick_x * 0.5;

        bot.moveBot(drive, turn, strafe, 0.5);


        telemetry.addData("Left Switch", bot.leftFrontSwitch.getState());
        telemetry.addData("Right Switch", bot.rightFrontSwitch.getState());


        if (gamepad1.right_bumper)
            clawPos = Karen.CLAW_OPEN;
        if (gamepad1.left_bumper)
            clawPos = Karen.CLAW_CLOSED;

        //arm -----------------
        if (gamepad1.dpad_down) { // arm down
            armPos -= 10; // positive due to motor rotation flipped
            // Lowest arm can go for safety,
            if (armPos < Karen.MIN_ARM_POSITION){ // 40
                armPos = Karen.MIN_ARM_POSITION;
            }

            bot.moveArm(armPos);
            armPos = bot.getCurrentArmPos();
            telemetry.addData("arm down", "");
        } else if (gamepad1.dpad_up) {
            armPos += 10;

            if(armPos > Karen.MAX_ARM_POSITION){ // -365
                armPos = Karen.MAX_ARM_POSITION;
            }

            bot.moveArm(armPos);
            armPos = bot.getCurrentArmPos();
            telemetry.addData("arm up", "");
        } else {
            bot.moveArm(armPos);
            telemetry.addData("arm still", "");
        }


        //hold to center
        if(gamepad1.x){
            telemetry.addData("", bot.center());
        }




        bot.clawServo.setPosition(clawPos);

        telemetry.addData("Servo: ", clawPos);
        telemetry.addData("armPos:", bot.getCurrentArmPos());

        telemetry.addData("leftOdo: ", bot.leftEncoder.getCurrentPosition());
        telemetry.addData("rightOdo: ", bot.rightEncoder.getCurrentPosition());
        telemetry.addData("backOdo: ", bot.frontEncoder.getCurrentPosition());
       // telemetry.addData("leftMotor", bot.leftFrontMotor.getPower());
        telemetry.addData("leftEncoder", bot.leftEncoder.getCurrentPosition());
        telemetry.addData("rightEncoder", bot.rightEncoder.getCurrentPosition());
        telemetry.addData("frontEncoder", bot.frontEncoder.getCurrentPosition());
        telemetry.addData("leftEncoderCorVel", bot.leftEncoder.getCorrectedVelocity());
        telemetry.addData("rightEncoderCorVel", bot.rightEncoder.getCorrectedVelocity());
        telemetry.addData("frontEncoderCorVel", bot.frontEncoder.getCorrectedVelocity());
        telemetry.addData("leftEncoderRawVel", bot.leftEncoder.getRawVelocity());
        telemetry.addData("rightEncoderRawVel", bot.rightEncoder.getRawVelocity());
        telemetry.addData("frontEncoderRawVel", bot.frontEncoder.getRawVelocity());
    }


    @Override
    public void stop() {
        bot.stop();
    }

}






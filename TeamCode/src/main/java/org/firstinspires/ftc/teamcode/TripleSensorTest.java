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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link TripleSensorTest} illustrates how to use the Modern Robotics
 * Range Sensor, an Analog Ultrasonic Sensor, and a REV 2M TOF sensor
 *
 */
@TeleOp(name = "Triple Sensor Test", group = "Sensor")
//@Disabled   // comment out or remove this line to enable this opmode
public class TripleSensorTest extends LinearOpMode {

    ModernRoboticsI2cRangeSensor rangeSensor;
    AnalogInput sonarSensor;
    private DistanceSensor laserSensor;
    //Rev2mDistanceSensor laserSensor;

    @Override public void runOpMode() {

        // get a reference to our compass
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");
        sonarSensor = hardwareMap.analogInput.get("sonarSensor");
        laserSensor = hardwareMap.get(DistanceSensor.class, "laserSensor");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)laserSensor;

        // wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
            // Optical only works inside of about 5cm
            telemetry.addData("raw optical", rangeSensor.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
            // Using the below method gets the best of ultrasonic and optical
            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));

            // The analog sonar sensor is good for longer distances, it becomes non-responsive
            // inside of about 7in/20cm and locks at it's min value
            telemetry.addData("Sonar Range:", "%.2f cm", sonarDistance());

            // The REV 2M TOF sensor is a bit wonky - it usually reads about 2cm higher than actual,
            // except when it doesn't.
            telemetry.addData("deviceName",laserSensor.getDeviceName() );
            //telemetry.addData("range", String.format("%.01f mm", laserSensor.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", laserSensor.getDistance(DistanceUnit.CM)));


            // Rev2mDistanceSensor specific methods.
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));
            telemetry.update();
        }
    }
    public double sonarDistance (){
        // Returns distance from the sonar sensor over an average of 4 values
        // Trying to compensate for noise in the sensor
        // 75 is the scaling factor between voltage and distance in INCHES
        // based on data collected on 11/17/2018
        // Multiply by 2.54 to convert output to CM
        double average;
        average = sonarSensor.getVoltage();
        sleep(1);
        average = average + sonarSensor.getVoltage();
        sleep(1);
        average = average + sonarSensor.getVoltage();
        sleep(1);
        average = average + sonarSensor.getVoltage();
        return (average*75*2.54);
    }
}

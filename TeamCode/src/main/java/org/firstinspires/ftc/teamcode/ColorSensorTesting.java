package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="ColorSensorTest", group="Iterative Opmode")
public class ColorSensorTesting extends OpMode {

    ColorSensor colorsense;

    @Override
    public void init() {
        colorsense = hardwareMap.colorSensor.get("colorSensor");
    }
    @Override
    public void loop() {
        telemetry.addData("red: ", colorsense.red());
        telemetry.addData("green: ", colorsense.green());
        telemetry.addData("blue: ", colorsense.blue());

        telemetry.addData("alpha: ", colorsense.alpha());
        telemetry.addData("arg-b: ",colorsense.argb());
    }

}

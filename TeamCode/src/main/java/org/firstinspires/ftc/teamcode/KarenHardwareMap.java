package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class KarenHardwareMap  {

    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;
    public KarenHardwareMap (HardwareMap map) {
        DcMotorEx leftFrontMotor = map.get(DcMotorEx.class, "leftFrontMotor");
        DcMotorEx rightFrontMotor = map.get(DcMotorEx.class, "rightFrontMotor");
        DcMotorEx leftBackMotor = map.get(DcMotorEx.class, "leftBackMotor");
        DcMotorEx rightBackMotor = map.get(DcMotorEx.class, "rightBackMotor");
    }
}

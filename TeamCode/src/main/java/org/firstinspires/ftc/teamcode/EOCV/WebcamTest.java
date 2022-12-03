package org.firstinspires.ftc.teamcode.EOCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.EOCV.SimTesterZone.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.LinkedList;
import java.util.Queue;

@Autonomous(name = "Signal Sleeve Test")
public class WebcamTest extends LinearOpMode {

    SleeveDetection sleeveDetection;
    OpenCvCamera camera;

     double avgVals[] = new double[10];
//    Queue<Double> avgVals = new LinkedList<>();
    int count = 0;

    // Name of the Webcam to be set in the config
    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection(telemetry);
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1920,1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
//


        // used to avg colors
        double sum = 0;
        double avgValOverTen = 0;

        while (!isStarted()) {
            telemetry.addData("AvgColor: ", sleeveDetection.avgColorVal);
            telemetry.addData("Area: ", sleeveDetection.area);
            telemetry.addData("SumColors[0]", sleeveDetection.sumColorsAtZero);
            telemetry.addData("Parking: ", sleeveDetection.getPosition());
            telemetry.update();


            // shifts values to the right
            for(int i = avgVals.length - 1; i > 0; i--){
                avgVals[i] = avgVals[i - 1];
            }
            avgVals[0] = sleeveDetection.avgColorVal;

            sum = 0;
            for(int i = 0; i < avgVals.length; i++){
               sum += avgVals[i];
            }

            avgValOverTen = sum / avgVals.length;

            telemetry.addData("DampedAvg: ", avgValOverTen);



        }

        waitForStart();
    }
}
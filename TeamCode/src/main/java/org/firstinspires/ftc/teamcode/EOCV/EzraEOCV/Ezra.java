package org.firstinspires.ftc.teamcode.EOCV.EzraEOCV;

import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.VideoCapture;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;

public class Ezra extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat cameraImage) {
        Mat mat = new Mat();
        Point textAnchor = new Point(20, 50);
        Scalar green = new Scalar(0,255,0,255);
        Mat mask = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC1);
        Scalar lowerOrange = new Scalar(0.0, 141.0, 0.0);
        Scalar upperOrange = new Scalar(255.0, 230.0, 95.0);

        Imgproc.cvtColor(cameraImage, cameraImage, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(mat, lowerOrange, upperOrange, mask);
        Imgproc.putText(cameraImage, "hello", textAnchor, Imgproc.FONT_HERSHEY_COMPLEX, 1.5, green, 2);

        return cameraImage;
    }

}
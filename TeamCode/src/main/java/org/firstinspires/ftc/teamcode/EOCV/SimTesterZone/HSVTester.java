package org.firstinspires.ftc.teamcode.EOCV.SimTesterZone;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class HSVTester extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    static final Point ANCHOR_TOP_LEFT = new Point(640, 360);
    static final int width = 1;
    static final int height = 1;

    static final Rect BANANA_ROI = new Rect(
            new Point(ANCHOR_TOP_LEFT.x, ANCHOR_TOP_LEFT.y),
            new Point(ANCHOR_TOP_LEFT.x + width, ANCHOR_TOP_LEFT.y + height));

    static final Rect LEFT_ROI = new Rect(
            new Point(60, 35),
            new Point(120, 75));
    static final Rect RIGHT_ROI = new Rect(
            new Point(140, 35),
            new Point(200, 75));

    static double PERCENT_COLOR_THRESHOLD = 0.4;
    double bananaValue = 0;

    public HSVTester(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {

        // make this yellow
        Scalar lowHSV = new Scalar(10, 50, 70);
        Scalar highHSV = new Scalar(30, 255, 255);

        // threshold this for the yellow balue above
        //Core.inRange(mat, lowHSV, highHSV, mat);

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Mat banana = mat.submat(BANANA_ROI);
        // Imgproc.cvtColor(input, banana, Imgproc.COLOR_RGB2HSV);

        //Core.inRange(banana, lowHSV, highHSV, banana);

        // print the value at center
        telemetry.addData("HSV: ", Core.sumElems(banana).val[0] / BANANA_ROI.area());

        // RELEASE THE BANANA
        banana.release();

//        telemetry.addData("SumYellow", (int) Core.sumElems(banana).val[0]);
//        telemetry.addData("YellowPerc", Math.round(bananaValue * 100) + "%");

        telemetry.update();

        Scalar
                RED  = new Scalar(255, 0, 0);

        Imgproc.rectangle(mat, BANANA_ROI, RED);

        Imgproc.rectangle(mat, new Rect(new Point(ANCHOR_TOP_LEFT.x-10, ANCHOR_TOP_LEFT.y-10), new Point(ANCHOR_TOP_LEFT.x+10, ANCHOR_TOP_LEFT.y+10)), RED);
//
//        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
//
//        Scalar colorStone = new Scalar(255, 0, 0);
//        Scalar colorSkystone = new Scalar(0, 255, 0);
//
//        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorSkystone:colorStone);
//        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorSkystone:colorStone);

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_HSV2RGB);
        return mat;
    }

    // returns tru if the yellow reaches the threshold (pole is seen)
    public boolean isBananaSeen(){
        return bananaValue > PERCENT_COLOR_THRESHOLD;
    }

}
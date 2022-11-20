package org.firstinspires.ftc.teamcode.EOCV;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SleeveDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    static final Rect LEFT_ROI = new Rect(
            new Point(75, 200),
            new Point(425, 350)
    );

    static final Rect CENTER_ROI = new Rect(
            new Point(800, 200), // was 775
            new Point(1150, 350) // was 850
    );

    static final Rect RIGHT_ROI = new Rect(
            new Point(1450, 200),
            new Point(1800, 350)
    );

    static double PERCENT_COLOR_THRESHOLD = 0.4; // the threshold of purple when the camera sees the TSE
    public SleeveDetector(Telemetry t) { telemetry = t; }

//    public boolean barLeft;
//    public boolean barCenter;
//    public boolean barRight;
    public boolean barLeft;
    public boolean barCenter;
    public boolean barRight;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // find the yellow value (change this to purple)
        Scalar lowHSVGreen = new Scalar(115, 20, 50);
        Scalar highHSVGreen = new Scalar(180, 255, 255);

        Scalar lowHSVRed = new Scalar(0, 20, 50);
        Scalar highHSVRed = new Scalar(40, 255, 255);

        Scalar lowHSVBlue = new Scalar(40, 20, 50);
        Scalar highHSVBlue = new Scalar(80, 255, 255);




//        Mat center = mat.submat(CENTER_ROI);
//        Mat right = mat.submat(RIGHT_ROI);
        Core.inRange(mat, lowHSVGreen, highHSVGreen, mat);
        Mat left = mat.submat(LEFT_ROI);
        double greenPerc = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double greenTotal = Core.sumElems(left).val[0];
        left.release();

        Core.inRange(mat, lowHSVRed, highHSVRed, mat);
        left = mat.submat(LEFT_ROI);
        double redPerc = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double redTotal = Core.sumElems(left).val[0];
        left.release();

        Core.inRange(mat, lowHSVBlue, highHSVBlue, mat);
        left = mat.submat(LEFT_ROI);
        double bluePerc = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double blueTotal = Core.sumElems(left).val[0];
        left.release();


        telemetry.addData("Green raw val", greenTotal);
        telemetry.addData("Blue raw val", redTotal);
        telemetry.addData("Red raw val", blueTotal);

        telemetry.addData("Green percentage", Math.round(greenPerc * 100) + "%");
        telemetry.addData("Blue percentage", Math.round(redPerc * 100) + "%");
        telemetry.addData("Red percentage", Math.round(bluePerc * 100) + "%");


//        double maxOfVals = Math.max(redPerc, Math.max(greenPerc, bluePerc));
//
//        barLeft = (maxOfVals == greenPerc);
//        barCenter = (maxOfVals == redPerc);
//        barRight = (maxOfVals == bluePerc);
//
//        if(barLeft){
//            telemetry.addData("Barcode:", " Left");
//        }
//
//        if(barCenter){
//            telemetry.addData("Barcode:", " Center");
//        }
//
//        if(barRight){
//            telemetry.addData("Barcode:", " Right");
//        }
//        telemetry.update();
//
//        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
//
//        Scalar onBarcode = new Scalar(200, 0, 255); // purple cuz purple team shipping element (aesthetic)
//        Scalar notOnBarcode = new Scalar(255, 0, 0);
//
//        Imgproc.rectangle(mat, LEFT_ROI, barLeft? onBarcode:notOnBarcode, 4);
//        Imgproc.rectangle(mat, CENTER_ROI, barCenter? onBarcode:notOnBarcode, 4);
//        Imgproc.rectangle(mat, RIGHT_ROI, barRight? onBarcode:notOnBarcode, 4);

        return mat;
    }


}
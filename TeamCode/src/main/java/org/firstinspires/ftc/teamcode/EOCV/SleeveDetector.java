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

    public boolean barLeft;
    public boolean barCenter;
    public boolean barRight;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // find the yellow value (change this to purple)
        Scalar lowHSV = new Scalar(115, 20, 50);
        Scalar highHSV = new Scalar(180, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat center = mat.submat(CENTER_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftTotal = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double centerTotal = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255;
        double rightTotal = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        center.release();
        right.release();

        telemetry.addData("Left raw val", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Center raw val", (int) Core.sumElems(center).val[0]);
        telemetry.addData("Right raw val", (int) Core.sumElems(right).val[0]);

        telemetry.addData("Left percentage", Math.round(leftTotal * 100) + "%");
        telemetry.addData("center percentage", Math.round(centerTotal * 100) + "%");
        telemetry.addData("right percentage", Math.round(rightTotal * 100) + "%");


        double maxOfVals = Math.max(centerTotal, Math.max(leftTotal, rightTotal));

        barLeft = (maxOfVals == leftTotal);
        barCenter = (maxOfVals == centerTotal);
        barRight = (maxOfVals == rightTotal);

        if(barLeft){
            telemetry.addData("Barcode:", " Left");
        }

        if(barCenter){
            telemetry.addData("Barcode:", " Center");
        }

        if(barRight){
            telemetry.addData("Barcode:", " Right");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar onBarcode = new Scalar(200, 0, 255); // purple cuz purple team shipping element (aesthetic)
        Scalar notOnBarcode = new Scalar(255, 0, 0);

        Imgproc.rectangle(mat, LEFT_ROI, barLeft? onBarcode:notOnBarcode, 4);
        Imgproc.rectangle(mat, CENTER_ROI, barCenter? onBarcode:notOnBarcode, 4);
        Imgproc.rectangle(mat, RIGHT_ROI, barRight? onBarcode:notOnBarcode, 4);

        return mat;
    }


}
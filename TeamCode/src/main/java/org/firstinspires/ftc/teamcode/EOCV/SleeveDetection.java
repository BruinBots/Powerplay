package org.firstinspires.ftc.teamcode.EOCV;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SleeveDetection extends OpenCvPipeline {
    Telemetry telemetry;
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */

    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    public double avgColorVal;
    public double area;
    public double sumColorsAtZero;


    // TOPLEFT anchor point for the bounding box
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(975, 450);

    // Width and height for the bounding box
    public static int REGION_WIDTH = 1;
    public static int REGION_HEIGHT = 1;

    // Color definitions
    private final Scalar
            RED  = new Scalar(255, 0, 0),
            GREEN    = new Scalar(0, 255, 0),
            BLUE = new Scalar(0, 0, 255);

    public double minRed = 150;
    public double maxRed = 180;
    public double minPurple = 90;
    public double maxPurple = 120;
    public double minGreen = 40; // found by dividing enriques vals by 2
    public double maxGreen = 70;


    // Anchor point definitions
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Running variable storing the parking position
    private volatile ParkingPosition position = ParkingPosition.LEFT;

    public SleeveDetection(Telemetry t) { telemetry = t;}

    @Override
    public Mat processFrame(Mat input) {
        // Get the submat frame, and then sum all the values

        final Rect BOUNDING_BOX = new Rect(sleeve_pointA, sleeve_pointB);
        Mat areaMat = input.submat(BOUNDING_BOX);

        Imgproc.cvtColor(input, areaMat, Imgproc.COLOR_RGB2HSV); //  converting to hsv
        Scalar sumColors = Core.sumElems(areaMat);

        sumColorsAtZero = sumColors.val[0];
        avgColorVal = sumColors.val[0] / BOUNDING_BOX.area() / 255;
        area = BOUNDING_BOX.area();


        telemetry.addData("BoundBoxArea: ", BOUNDING_BOX.area());
        telemetry.addData("AvgColor: ", avgColorVal);


        // Get the minimum RGB value from every single channel
        // double minColor = Math.min(sumColors.val[0], Math.min(sumColors.val[1], sumColors.val[2]));



        // Change the bounding box color based on the sleeve color

        // red == left = one dot
        if (avgColorVal < maxRed && avgColorVal > minRed) {
            position = ParkingPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    RED,
                    4
            );

            // green
        } else if (avgColorVal < maxGreen && avgColorVal > minGreen) {
            position = ParkingPosition.CENTER;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    GREEN,
                    4
            );

            // blue
        } else {
            position = ParkingPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    BLUE,
                    4
            );
        }

        // Release and return input
        areaMat.release();
        return input;
    }

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }
}
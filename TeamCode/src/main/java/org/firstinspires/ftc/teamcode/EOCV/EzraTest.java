package org.firstinspires.ftc.teamcode.EOCV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.VideoCapture;
import org.opencv.imgproc.Imgproc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class EzraTest extends LinearOpMode {

    static {
        // Load the OpenCV library
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    public void runOpMode() {
        // Wait for the start button to be pressed
        waitForStart();

        // Take a picture
        Mat image = takePicture();

        // Apply a mask to the image
        Mat maskedImage = applyMask(image);

        // Save the masked image to a file
        saveImage(maskedImage, "masked_image.png");
    }

    // This method takes a picture and returns the resulting image as a Mat
    public Mat takePicture() {
        // Initialize the video capture device
        VideoCapture capture = new VideoCapture(0);

        // Check if the capture device fails to open
        if (!capture.isOpened()) {
            throw new RuntimeException("Error opening capture device");
        }

        // Read a frame from the capture device
        Mat image = new Mat();
        capture.read(image);

        // Release the capture device
        capture.release();

        return image;
    }

    // This method applies a mask to the input image and returns the resulting image
    public Mat applyMask(Mat image) {
        Mat hsvImage = new Mat();
        Mat mask = new Mat();

        // Convert the image to the HSV color space
        Imgproc.cvtColor(image, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Set the lower and upper bounds for the colors you want to keep
        Scalar lowerBound = new Scalar(0, 100, 100);
        Scalar upperBound = new Scalar(10, 255, 255);

        // Use the inRange function to create a mask for the colors within the bounds
        Core.inRange(hsvImage, lowerBound, upperBound, mask);

        // Apply the mask to the image
        Core.bitwise_and(image, image, image, mask);

        return image;
    }

    // This method saves the input image to a file
    public void saveImage(Mat image, String fileName) {
        // Save the image to a file
        Imgcodecs.imwrite(fileName, image);
    }
}

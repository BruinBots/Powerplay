package org.firstinspires.ftc.teamcode.EOCV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.VideoCapture;

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
        // Create a mask with all values set to 0
        Mat mask = new Mat(image.rows(), image.cols(), image.type(), new Scalar(0));

        // Apply the mask to the image
        Core.bitwise_and(image, mask, image);

        return image;
    }

    // This method saves the input image to a file
    public void saveImage(Mat image, String fileName) {
        // Save the image to a file
        Imgcodecs.imwrite(fileName, image);
    }
}
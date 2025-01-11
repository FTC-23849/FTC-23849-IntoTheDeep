// Pipeline class for OpenCV processing
package org.firstinspires.ftc.teamcode.Auto;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SampleAlignmentPipeline2 extends OpenCvPipeline {

    private double sampleAngle = 0.0;

    // Pre-allocated Mats to avoid re-allocation
    private final Mat hsv = new Mat();
    private final Mat maskYellow = new Mat();
    private final Mat maskBlue = new Mat();
    private final Mat maskRed = new Mat();
    private final Mat maskRed2 = new Mat();
    private final Mat combinedMask = new Mat();
    private final Mat hierarchy = new Mat();
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

    public double getSampleAngle() {
        return sampleAngle;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Convert to HSV color space
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

        // Define HSV ranges for yellow, blue, and red
        Scalar lowerYellow = new Scalar(15, 70, 70);
        Scalar upperYellow = new Scalar(35, 255, 255);
        Scalar lowerBlue = new Scalar(90, 100, 70);
        Scalar upperBlue = new Scalar(130, 255, 255);
        Scalar lowerRed1 = new Scalar(0, 100, 100);
        Scalar upperRed1 = new Scalar(10, 255, 255);
        Scalar lowerRed2 = new Scalar(160, 100, 100);
        Scalar upperRed2 = new Scalar(180, 255, 255);

        // Create masks for each color
        Core.inRange(hsv, lowerYellow, upperYellow, maskYellow);
        Core.inRange(hsv, lowerBlue, upperBlue, maskBlue);
        Core.inRange(hsv, lowerRed1, upperRed1, maskRed);
        Core.inRange(hsv, lowerRed2, upperRed2, maskRed2);
        Core.add(maskRed, maskRed2, maskRed);

        // Combine masks
        Core.add(maskYellow, maskBlue, combinedMask);
        Core.add(combinedMask, maskRed, combinedMask);

        // Apply morphological operations to clean up the mask
        Imgproc.morphologyEx(combinedMask, combinedMask, Imgproc.MORPH_CLOSE, kernel);

        // Find contours on the combined mask
        java.util.List<MatOfPoint> contours = new java.util.ArrayList<>();
        Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Calculate the middle point of the frame
        Point frameCenter = new Point(input.cols() / 2.0, input.rows() / 2.0);

        // Draw a dot at the frame center
        Imgproc.circle(input, frameCenter, 5, new Scalar(0, 255, 0), -1); // Green dot, 10-pixel diameter

        // Variables to store the selected sample's data
        RotatedRect bestSample = null;

        for (MatOfPoint contour : contours) {
            // Approximate contour to a polygon
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
            contour2f.release(); // Release memory for contour2f

            // Check if the middle point of the frame is within the sample
            if (rotatedRect.boundingRect().contains(frameCenter)) {
                bestSample = rotatedRect;
                break; // Found the desired sample, no need to check further
            }
        }

        if (bestSample != null) {
            double width = bestSample.size.width;
            double height = bestSample.size.height;
            if (width < height) {
                sampleAngle = 180 - bestSample.angle;
                if (sampleAngle >= 180) {
                    sampleAngle -= 180;
                }
            } else {
                sampleAngle = 180 - ((bestSample.angle + 90) % 180);
                if (sampleAngle >= 180) {
                    sampleAngle -= 180;
                }
            }

            // Draw the contour and bounding box of the selected sample
            Point[] boxPoints = new Point[4];
            bestSample.points(boxPoints);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(255, 0, 0), 2);
            }
        }

        return input;
    }

    @Override
    public void finalize() {
        // Release pre-allocated Mats
        hsv.release();
        maskYellow.release();
        maskBlue.release();
        maskRed.release();
        maskRed2.release();
        combinedMask.release();
        hierarchy.release();
        kernel.release();
    }
}

package org.firstinspires.ftc.teamcode;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SampleAlignmentPipeline7 extends OpenCvPipeline {

    private double sampleAngle = 0.0;
    private String sampleColor = "Unknown";

    private final Mat hsv = new Mat();
    private final Mat maskYellow = new Mat();
    private final Mat maskBlue = new Mat();
    private final Mat maskRed = new Mat();
    private final Mat maskRed2 = new Mat();
    private final Mat edgeMask = new Mat();
    private final Mat combinedMask = new Mat();
    private final Mat hierarchy = new Mat();
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));

    public double getSampleAngle() {
        return sampleAngle;
    }

    public String getSampleColor() {
        return sampleColor;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

        // Define HSV ranges for red, yellow, and blue
        Scalar lowerYellow = new Scalar(10, 40, 50);
        Scalar upperYellow = new Scalar(45, 255, 255);
        Scalar lowerBlue = new Scalar(85, 70, 50);
        Scalar upperBlue = new Scalar(135, 255, 255);
        Scalar lowerRed1 = new Scalar(0, 60, 50);
        Scalar upperRed1 = new Scalar(10, 255, 255);
        Scalar lowerRed2 = new Scalar(160, 60, 50);
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

        // Detect edges based on color transitions
        detectSampleEdges();

        // Apply morphological operations to clean up edges
        Imgproc.morphologyEx(edgeMask, edgeMask, Imgproc.MORPH_CLOSE, kernel);

        // Find contours using edge detection mask
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(edgeMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Define bounding box size constraints
        double frameArea = input.cols() * input.rows();
        double minSampleArea = frameArea / 10.0;  // Slightly smaller samples allowed
        double maxSampleArea = frameArea / 2.2;

        // Find the sample at the center of the frame
        Point frameCenter = new Point(input.cols() / 2.0, input.rows() / 2.0);
        RotatedRect bestSample = null;

        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
            contour2f.release();

            double width = rotatedRect.size.width;
            double height = rotatedRect.size.height;
            double area = width * height;

            // Ensure bounding box is within size limits
            if (area < minSampleArea || area > maxSampleArea) {
                continue;
            }

            // Check if the bounding box contains the center point
            if (rotatedRect.boundingRect().contains(frameCenter)) {
                bestSample = rotatedRect;

                // Determine sample color
                Mat mask = Mat.zeros(input.size(), CvType.CV_8UC1);
                Imgproc.drawContours(mask, contours, contours.indexOf(contour), new Scalar(255), -1);
                Scalar avgColor = Core.mean(hsv, mask);

                if (avgColor.val[0] >= 10 && avgColor.val[0] <= 45) {
                    sampleColor = "Yellow";
                } else if (avgColor.val[0] >= 85 && avgColor.val[0] <= 135) {
                    sampleColor = "Blue";
                } else if ((avgColor.val[0] >= 0 && avgColor.val[0] <= 10) || (avgColor.val[0] >= 160 && avgColor.val[0] <= 180)) {
                    sampleColor = "Red";
                } else {
                    sampleColor = "Unknown";
                }

                mask.release();
                break;
            }
        }

        if (bestSample != null) {
            double width = bestSample.size.width;
            double height = bestSample.size.height;
            sampleAngle = (width < height) ? (180 - bestSample.angle) : (180 - ((bestSample.angle + 90) % 180));

            // Draw the bounding box around detected sample
            Point[] boxPoints = new Point[4];
            bestSample.points(boxPoints);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(255, 0, 0), 2);
            }
        } else {
            sampleAngle = 0.0;
            sampleColor = "Unknown";
        }

        // Draw the detected edges in green
        Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0), 1);

        return input;
    }

    /**
     * Detect edges of samples based on color transitions between Red, Yellow, and Blue.
     */
    private void detectSampleEdges() {
        edgeMask.setTo(new Scalar(0));

        for (int y = 1; y < hsv.rows() - 1; y++) {
            for (int x = 1; x < hsv.cols() - 1; x++) {
                double[] currPixel = hsv.get(y, x);
                double[] rightPixel = hsv.get(y, x + 1);
                double[] bottomPixel = hsv.get(y + 1, x);

                // Detect horizontal and vertical color transitions
                if (isColorTransition(currPixel, rightPixel) || isColorTransition(currPixel, bottomPixel)) {
                    edgeMask.put(y, x, 255);
                }
            }
        }
    }

    /**
     * Checks if two adjacent pixels belong to different sample colors.
     */
    private boolean isColorTransition(double[] p1, double[] p2) {
        return (isRed(p1) && !isRed(p2)) || (isYellow(p1) && !isYellow(p2)) || (isBlue(p1) && !isBlue(p2));
    }

    private boolean isRed(double[] pixel) {
        return (pixel[0] >= 0 && pixel[0] <= 10) || (pixel[0] >= 160 && pixel[0] <= 180);
    }

    private boolean isYellow(double[] pixel) {
        return pixel[0] >= 10 && pixel[0] <= 45;
    }

    private boolean isBlue(double[] pixel) {
        return pixel[0] >= 85 && pixel[0] <= 135;
    }

    @Override
    public void finalize() {
        hsv.release();
        maskYellow.release();
        maskBlue.release();
        maskRed.release();
        maskRed2.release();
        combinedMask.release();
        edgeMask.release();
        hierarchy.release();
        kernel.release();
    }
}

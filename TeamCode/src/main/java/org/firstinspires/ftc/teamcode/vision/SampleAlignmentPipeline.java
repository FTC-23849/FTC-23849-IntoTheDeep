package org.firstinspires.ftc.teamcode.vision;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SampleAlignmentPipeline extends OpenCvPipeline {

    private double sampleAngle = 0.0;

    public double getSampleAngle() {
        return sampleAngle;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat gray = new Mat();
        Mat blurred = new Mat();
        Mat edges = new Mat();

        // Convert to grayscale
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);

        // Apply Gaussian blur to reduce noise
        Imgproc.GaussianBlur(gray, blurred, new Size(5, 5), 0);

        // Detect edges using Canny
        Imgproc.Canny(blurred, edges, 50, 150);

        // Find contours
        java.util.List<MatOfPoint> contours = new java.util.ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Define ROI rectangle (adjust coordinates as necessary)
        Rect roi = new Rect(input.cols() / 4, input.rows() / 4, input.cols() / 2, input.rows() / 2);
        Imgproc.rectangle(input, roi, new Scalar(0, 255, 0), 2); // Draw ROI rectangle on input frame
        Mat roiMat = edges.submat(roi);

        // Variables to store the selected sample's data
        RotatedRect bestSample = null;
        double minCenterDistance = Double.MAX_VALUE;

        for (MatOfPoint contour : contours) {
            // Approximate contour to a polygon
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);

            // Compute aspect ratio
            double width = rotatedRect.size.width;
            double height = rotatedRect.size.height;
            double aspectRatio = Math.max(width, height) / Math.min(width, height);

            // Check if the aspect ratio matches a rectangular sample (adjust threshold as needed)
            if (aspectRatio > 2.0 && aspectRatio < 3.0) {
                Point center = rotatedRect.center;

                // Check if the center is within the ROI
                if (roi.contains(center)) {
                    double centerDistance = Math.sqrt(
                            Math.pow(center.x - (roi.x + roi.width / 2), 2) +
                                    Math.pow(center.y - (roi.y + roi.height / 2), 2)
                    );

                    // Select the sample closest to the ROI center
                    if (centerDistance < minCenterDistance) {
                        minCenterDistance = centerDistance;
                        bestSample = rotatedRect;
                    }
                }
            }
        }

        if (bestSample != null) {
            sampleAngle = bestSample.angle;

            // Draw the contour and bounding box of the selected sample
            Point[] boxPoints = new Point[4];
            bestSample.points(boxPoints);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(255, 0, 0), 2);
            }
        }

        // Release memory
        gray.release();
        blurred.release();
        edges.release();
        hierarchy.release();

        return input;
    }
}
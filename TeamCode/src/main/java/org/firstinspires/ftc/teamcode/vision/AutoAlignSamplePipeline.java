//package org.firstinspires.ftc.teamcode.Auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.MatOfPoint2f;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.RotatedRect;
//import org.opencv.core.Size;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.ArrayList;
//import java.util.List;
//
//public class AutoAlignSamplePipeline extends OpenCvPipeline {
//
//    public static double detectAndAlignClawInROI(Mat frame /*, Servo clawServo*/, Rect roi) {
//        // Downscale ROI for faster processing
//        final double SCALE_FACTOR = 0.5; // Adjust as needed
//        Mat roiMat = new Mat(frame, roi);
//        Mat resizedROI = new Mat();
//        Imgproc.resize(roiMat, resizedROI, new Size(roi.width * SCALE_FACTOR, roi.height * SCALE_FACTOR));
//
//        try {
//            // Convert resized ROI to grayscale
//            Mat gray = new Mat();
//            Imgproc.cvtColor(resizedROI, gray, Imgproc.COLOR_BGR2GRAY);
//
//            // Threshold the grayscale image
//            Mat thresholded = new Mat();
//            Imgproc.threshold(gray, thresholded, 127, 255, Imgproc.THRESH_BINARY);
//
//            // Find contours
//            List<MatOfPoint> contours = new ArrayList<>();
//            Mat hierarchy = new Mat();
//            Imgproc.findContours(thresholded, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//
//            if (contours.isEmpty()) {
//                System.out.println("No contours detected in ROI.");
//                return 0.0;
//            }
//
//            // Calculate ROI center in the resized frame
//            Point roiCenter = new Point(resizedROI.cols() / 2.0, resizedROI.rows() / 2.0);
//
//            double closestDistance = Double.MAX_VALUE;
//            MatOfPoint closestContour = null;
//
//            for (MatOfPoint contour : contours) {
//                Rect boundingRect = Imgproc.boundingRect(contour);
//
//                // Early filter by bounding box size (to eliminate noise)
//                if (boundingRect.width < 10 || boundingRect.height < 10) continue;
//
//                // Calculate aspect ratio and area
//                double aspectRatio = (double) boundingRect.width / boundingRect.height;
//                double contourArea = Imgproc.contourArea(contour);
//                double roiArea = resizedROI.cols() * resizedROI.rows();
//
//                // Filter by aspect ratio (±20% tolerance) and relative area
//                if (aspectRatio < 2.33 * 0.8 || aspectRatio > 2.33 * 1.2) continue;
//                if (contourArea < 0.005 * roiArea || contourArea > 0.5 * roiArea) continue;
//
//                // Calculate contour center and distance to ROI center
//                Point contourCenter = new Point(
//                        boundingRect.x + boundingRect.width / 2.0,
//                        boundingRect.y + boundingRect.height / 2.0
//                );
//                double distance = Math.sqrt(Math.pow(contourCenter.x - roiCenter.x, 2) + Math.pow(contourCenter.y - roiCenter.y, 2));
//
//                // Find the closest contour to the ROI center
//                if (distance < closestDistance) {
//                    closestDistance = distance;
//                    closestContour = contour;
//                }
//            }
//
//            if (closestContour == null) {
//                System.out.println("No valid contour found near ROI center.");
//                return 0.0;
//            }
//
//            // Get the rotated rectangle for the closest contour
//            MatOfPoint2f contour2f = new MatOfPoint2f(closestContour.toArray());
//            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
//
//            // Calculate the angle of the rectangle
//            double angle = rotatedRect.angle;
//            if (rotatedRect.size.width < rotatedRect.size.height) {
//                angle += 90;
//            }
//
//            // Align the claw servo to the detected angle
////            rotateClawServo(clawServo, angle);
//
//            return angle;
//        } finally {
//            // Release all resources to prevent memory leaks
//            resizedROI.release();
//            roiMat.release();
//        }
//    }
//
//
//    @Override
//    public Mat processFrame(Mat input) {
//        return null;
//    }
//}

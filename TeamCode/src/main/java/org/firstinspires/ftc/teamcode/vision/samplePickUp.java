package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.rrfiles.MecanumDrive;

import org.firstinspires.ftc.teamcode.Robot;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class samplePickUp extends OpenCvPipeline {

    private double sampleAngle = 0.0;
    private String zone = null;
    private String side = null;
    private static final int CENTER_MARGIN_WIDTH = 50;  // Adjust width of the centerMargin rectangle
    private static final double TARGETPOINT_SCALAR = 1.65;


    Mat outPut = new Mat();


    @Override
    public Mat processFrame(Mat input) {

        Mat redMask = createRedMask(input);
        Mat blueMask = createBlueMask(input);
        Mat yellowMask = createYellowMask(input);

        Mat combinedMask = new Mat();
        Core.bitwise_or(redMask, yellowMask, combinedMask); // Combine red and yellow masks
        Core.bitwise_or(combinedMask, blueMask, combinedMask); // Now combine with blue mask


        // Find contours of the detected yellow regions
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);



        // Calculate the target point of the frame
        Point targetPoint = new Point(input.cols() / 2.0, input.rows() / TARGETPOINT_SCALAR);

        // Draw a dot at the target point
        Imgproc.circle(input, targetPoint, 10, new Scalar(0, 255, 0), -1); // Green dot, 10-pixel diameter



        // Define the two rectangles (splitting the frame vertically)
        Rect leftRect = new Rect(0, 0, input.cols() / 2, input.rows());
        Rect rightRect = new Rect(input.cols() / 2, 0, input.cols() / 2, input.rows());

        // Draw rectangles on the frame
        Imgproc.rectangle(input, leftRect.tl(), leftRect.br(), new Scalar(0, 0, 255), 3); // Blue
        Imgproc.rectangle(input, rightRect.tl(), rightRect.br(), new Scalar(0, 255, 0), 3); // Green



        // Calculate the top-left and bottom-right points for the centered centerMargin rectangle
        int centerMarginHeight = input.rows(); // Full height of the frame

        // Calculate the top-left point for the centered skinny rectangle
        int centerMarginX = (input.cols() - CENTER_MARGIN_WIDTH) / 2;
        int centerMarginY = 0;

        // Create the skinny rectangle object
        Rect centerMargin = new Rect(centerMarginX, centerMarginY, CENTER_MARGIN_WIDTH, centerMarginHeight);

        // Draw the centerMargin
        Imgproc.rectangle(input, centerMargin.tl(), centerMargin.br(), new Scalar(255, 255, 255), 3); // Blue



        // Define the height of the horizontal rectangle
        int horizontalCenterMarginHeight = 50; // Adjust thickness if needed

        // Create the rectangle (x, y, width, height)
        Rect horizontalCenterMargin = new Rect(0, (int) (targetPoint.y - horizontalCenterMarginHeight / 2), input.cols(), horizontalCenterMarginHeight);

        // Draw the horizontal rectangle in white
        Imgproc.rectangle(input, horizontalCenterMargin, new Scalar(255, 255, 255), 3);



        Rect topRect = new Rect(0, 0, input.cols(), (int) (targetPoint.y));

        // Bottom rectangle (from bottom of frame up to frameCenterY)
        Rect bottomRect = new Rect(0, (int) (targetPoint.y), input.cols(), input.rows() - (int) (targetPoint.y));

        Imgproc.rectangle(input, topRect, new Scalar(255, 215, 0), 3); // Gold
        Imgproc.rectangle(input, bottomRect, new Scalar(0, 0, 255), 3); // Red


        // Find the contour closest to the center
        MatOfPoint closestContour = null;
        double minDistance = Double.MAX_VALUE;
        Point closestCentroid = null;  // Correctly storing the closest centroid

        for (MatOfPoint contour : contours) {
            if (Imgproc.contourArea(contour) > 900) {  // Ignore small contours
                // Compute the centroid
                Moments moments = Imgproc.moments(contour);
                if (moments.get_m00() != 0) {  // Avoid division by zero
                    double cx = moments.get_m10() / moments.get_m00();
                    double cy = moments.get_m01() / moments.get_m00();
                    Point centroid = new Point(cx, cy);  // Centroid of this contour

                    // Compute the Euclidean distance to the frame center
                    double distance = Math.hypot(targetPoint.x - cx, targetPoint.y - cy);

                    // Check if this is the closest contour found so far
                    if (distance < minDistance) {
                        minDistance = distance;
                        closestContour = contour;
                        closestCentroid = centroid;  // Store the correct centroid
                    }
                }
            }
        }

        RotatedRect bestSample = null;

        // Draw only the closest contour
        if (closestContour != null) {
            // Draw a bounding box around the closest contour
            RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(closestContour.toArray()));
            Point[] rectPoints = new Point[4];
            rotatedRect.points(rectPoints);
            Imgproc.polylines(input, Arrays.asList(new MatOfPoint(rectPoints)), true, new Scalar(0, 255, 0), 2);

            bestSample = rotatedRect;

            double width = bestSample.size.width;
            double height = bestSample.size.height;

            if (width < height) {
                sampleAngle = (180 - bestSample.angle + 90) % 180;  // Corrected for rotated frame
            } else {
                sampleAngle = (180 - ((bestSample.angle + 90) % 180) + 90) % 180;
            }

            /*if (width < height) {
                sampleAngle = 180 - bestSample.angle;
            } else {
                sampleAngle = 180 - ((bestSample.angle + 90) % 180);
            }*/

            // Draw a small circle at the centroid of the closest contour
            if (closestCentroid != null) {
                if (closestCentroid.inside(centerMargin)) {
                    side = "aligned";
                } else if (closestCentroid.inside(rightRect)) {
                    side = "right";
                } else if (closestCentroid.inside(leftRect)){
                    side = "left";
                }

                if (closestCentroid.inside(horizontalCenterMargin)) {
                    zone = "aligned";
                } else if (closestCentroid.inside(topRect)) {
                    zone = "top";
                } else if (closestCentroid.inside(bottomRect)){
                    zone = "bottom";
                }
                Imgproc.circle(input, closestCentroid, 5, new Scalar(255, 0, 0), -1); // Blue dot for centroid
            }
        }

        input.copyTo(outPut);

        redMask.release();
        blueMask.release();
        yellowMask.release();
        combinedMask.release();
        hierarchy.release();


        return outPut;

    }

    private Mat createRedMask(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        Scalar lowerRed = new Scalar(110, 100, 100);
        Scalar upperRed = new Scalar(180, 255, 255);

        Mat redMask = new Mat();
        Core.inRange(hsvFrame, lowerRed, upperRed, redMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_CLOSE, kernel);

        hsvFrame.release();
        kernel.release();

        return redMask;
    }

    private Mat createBlueMask(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);

        Scalar lowerBlue = new Scalar(100, 150, 150);
        Scalar upperBlue = new Scalar(130, 255, 255);

        Mat blueMask = new Mat();
        Core.inRange(hsvFrame, lowerBlue, upperBlue, blueMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_CLOSE, kernel);

        hsvFrame.release();
        kernel.release();

        return blueMask;
    }

    private Mat createYellowMask(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);

        Scalar lowerYellow = new Scalar(20, 100, 100);
        Scalar upperYellow = new Scalar(30, 255, 255);

        Mat yellowMask = new Mat();
        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

        hsvFrame.release();
        kernel.release();

        return yellowMask;
    }

    public double getCurrAngle() {
        return sampleAngle;
    }

    public String getZone() {
        return zone;
    }

    public String getSide() {
        return side;
    }

    public void alignToSampleHorizontal(DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, double power, MecanumDrive drive) {
        if (side == "aligned") {

        } else if (side == "left") {

            while (side != "aligned") {
                drive.updatePoseEstimate();
                drive.localizer.update();
                leftFront.setPower(-power);
                leftBack.setPower(power);
                rightFront.setPower(power);
                rightBack.setPower(-power);
            }

            leftFront.setPower(0.0);
            leftBack.setPower(0.0);
            rightFront.setPower(0.0);
            rightBack.setPower(0.0);

            drive.updatePoseEstimate();
            drive.localizer.update();

        } else if (side == "right") {

            while (side != "aligned") {
                drive.updatePoseEstimate();
                drive.localizer.update();
                leftFront.setPower(power);
                leftBack.setPower(-power);
                rightFront.setPower(-power);
                rightBack.setPower(power);
            }

            leftFront.setPower(0.0);
            leftBack.setPower(0.0);
            rightFront.setPower(0.0);
            rightBack.setPower(0.0);

            drive.updatePoseEstimate();
            drive.localizer.update();

        }

    }

    public void alignToSampleVertical(DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, double power, MecanumDrive drive) {
        if (zone == "aligned") {

        } else if (zone == "top") {

            while (zone != "aligned") {
                drive.updatePoseEstimate();
                drive.localizer.update();
                leftFront.setPower(power);
                leftBack.setPower(power);
                rightFront.setPower(power);
                rightBack.setPower(power);
            }

            leftFront.setPower(0.0);
            leftBack.setPower(0.0);
            rightFront.setPower(0.0);
            rightBack.setPower(0.0);

            drive.updatePoseEstimate();
            drive.localizer.update();

        } else if (zone == "bottom") {

            while (zone != "aligned") {
                drive.updatePoseEstimate();
                drive.localizer.update();
                leftFront.setPower(-power);
                leftBack.setPower(-power);
                rightFront.setPower(-power);
                rightBack.setPower(-power);
            }

            leftFront.setPower(0.0);
            leftBack.setPower(0.0);
            rightFront.setPower(0.0);
            rightBack.setPower(0.0);

            drive.updatePoseEstimate();
            drive.localizer.update();

        }

    }

    public void clawAlign(Servo intakeDiffyLeft, Servo intakeDiffyRight) {
        if(sampleAngle <= 22.5){
            intakeDiffyLeft.setPosition(Robot.INTAKE_LEFT_DIFFY_PICK_UP_VERTICAL);
            intakeDiffyRight.setPosition(Robot.INTAKE_RIGHT_DIFFY_PICK_UP_VERTICAL);

        }
        else if(sampleAngle >22.5 && sampleAngle <= 67.5 ){
            intakeDiffyLeft.setPosition(Robot.INTAKE_LEFT_DIFFY_PICK_UP_45);
            intakeDiffyRight.setPosition(Robot.INTAKE_RIGHT_DIFFY_PICK_UP_45);

        }
        else if (sampleAngle >67.5 && sampleAngle <= 112.5){
            intakeDiffyLeft.setPosition(Robot.INTAKE_LEFT_DIFFY_PICK_UP);
            intakeDiffyRight.setPosition(Robot.INTAKE_RIGHT_DIFFY_PICK_UP);
        }
        else if (sampleAngle >112.5 && sampleAngle <= 157.5){
            intakeDiffyLeft.setPosition(Robot.INTAKE_LEFT_DIFFY_PICK_UP_135);
            intakeDiffyRight.setPosition(Robot.INTAKE_RIGHT_DIFFY_PICK_UP_135);
        }
        else {
            intakeDiffyLeft.setPosition(Robot.INTAKE_LEFT_DIFFY_PICK_UP_VERTICAL);
            intakeDiffyRight.setPosition(Robot.INTAKE_RIGHT_DIFFY_PICK_UP_VERTICAL);

        }
    }

}

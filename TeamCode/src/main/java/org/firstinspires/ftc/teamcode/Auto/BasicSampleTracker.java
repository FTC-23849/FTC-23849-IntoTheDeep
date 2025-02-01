package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp
public class BasicSampleTracker extends LinearOpMode {

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution

    @Override
    public void runOpMode(){
        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        waitForStart();

        while (opModeIsActive()) {

        }
        controlHubCam.stopStreaming();
        controlHubCam.closeCameraDevice();

    }

    class stream extends OpenCvPipeline {

        //Mat HSVFrame = new Mat();

        Mat outPut = new Mat();

        //Scalar rectColor = new Scalar(255.0, 0.0, 0.0);

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

            // Calculate the middle point of the frame
            Point frameCenter = new Point(input.cols() / 2.0, input.rows() / 2.0);

            // Draw a dot at the frame center
            Imgproc.circle(input, frameCenter, 5, new Scalar(0, 255, 0), -1); // Green dot, 10-pixel diameter

            for (MatOfPoint contour : contours) {

                // Filter out small contours based on area
                if (Imgproc.contourArea(contour) > 1000) {  // Adjust this threshold based on real-world testing
                    // Use minAreaRect to get the minimum enclosing rectangle
                    RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

                    // Get the four corners of the rectangle
                    Point[] rectPoints = new Point[4];
                    rotatedRect.points(rectPoints);

                    // Draw the parallelogram on the input image
                    Imgproc.polylines(input, Arrays.asList(new MatOfPoint(rectPoints)), true, new Scalar(0, 255, 0), 2);

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

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //controlHubCam.setPipeline(new ObjectTrackingCustom.YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
        controlHubCam.setPipeline(new stream());
    }

}
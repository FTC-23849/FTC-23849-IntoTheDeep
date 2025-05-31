package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@TeleOp(name = "SampleAlignmentTeleOp3", group = "FTC")
public class SampleAlignmentTeleOp3 extends OpMode {

    private OpenCvCamera camera;
    private SampleAlignmentPipeline3 pipeline;
//    private Servo clawServo;
    private boolean isCameraActive = false;
    private double lastServoPosition = -1; // Indicates no initial position
    private double trueServoPosition = 0.0; // Stores the actual value passed to setPosition

    @Override
    public void init() {
        // Initialize camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Create and set pipeline
        pipeline = new SampleAlignmentPipeline3();
        camera.setPipeline(pipeline);

        // Start camera streaming
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                isCameraActive = true;
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
            }
        });

        // FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(camera, 30);

        // Initialize servo
//        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    @Override
    public void loop() {
        double angle = pipeline.getSampleAngle(); // Retrieve the sample angle from the pipeline

        // Control the servo angle using the camera pipeline, but only when gamepad1.a is pressed
        if (gamepad1.a && isCameraActive) {
//            // Map the sample angle (0-180 degrees) to the servo position (0-300 degrees physical)
            trueServoPosition = angle / 300.0; // Normalize to 0-1 based on servo's 300-degree range
            trueServoPosition = Math.min(1.0, Math.max(0.0, trueServoPosition)); // Clamp to [0, 1]
//            clawServo.setPosition(trueServoPosition);
        }

        // Display telemetry data
        telemetry.addData("Sample Angle", angle); // Display sample angle
        telemetry.addData("Sample Color", pipeline.getSampleColor()); // Display sample color
        telemetry.addData("True Servo Position", trueServoPosition); // True servo position set
        telemetry.update();
    }

    @Override
    public void stop() {
        if (isCameraActive && camera != null) {
            try {
                // Stop streaming before closing the camera
                camera.stopStreaming();

                // Ensure camera closes asynchronously
                camera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
                    @Override
                    public void onClose() {
                        camera = null; // Ensure garbage collection removes it
                    }
                });

                isCameraActive = false;

                // Release OpenCV resources safely
                if (pipeline != null) {
                    pipeline.finalize();
                }
            } catch (Exception e) {
                telemetry.addData("Camera Stop Error", e.getMessage());
                telemetry.update();
            }
        }
    }
}


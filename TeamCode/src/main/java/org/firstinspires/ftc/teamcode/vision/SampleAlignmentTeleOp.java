package org.firstinspires.ftc.teamcode.vision;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp(name = "SampleAlignmentTeleOp", group = "FTC")
public class SampleAlignmentTeleOp extends OpMode {

    private OpenCvCamera camera;
    private SampleAlignmentPipeline pipeline;
    private Servo clawServo;

    @Override
    public void init() {
        // Initialize camera
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Create and set pipeline
        pipeline = new SampleAlignmentPipeline();
        camera.setPipeline(pipeline);

        // Set camera resolution and start
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
            }
        });

        // Initialize servo
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    @Override
    public void loop() {
        // Get the sample angle from the pipeline
        double angle = pipeline.getSampleAngle();

        // Map angle to servo position (adjust scaling as needed)
        double servoPosition = (angle + 90) / 180; // Map -90 to 90 degrees to 0 to 1
        clawServo.setPosition(servoPosition);

        // Display angle on telemetry
        telemetry.addData("Sample Angle", angle);
        telemetry.update();
    }

    @Override
    public void stop() {
        camera.stopStreaming();
        camera.closeCameraDevice();
    }
}

//package org.firstinspires.ftc.teamcode.Auto;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//import static org.firstinspires.ftc.teamcode.Auto.AutoAlignSamplePipeline.detectAndAlignClawInROI;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.opencv.core.Mat;
//import org.opencv.core.Rect;
//import org.openftc.easyopencv.*;
//
//
//
//@TeleOp(name = "GoBildaCameraExample")
//public class AutoAlignSample extends LinearOpMode {
//    OpenCvCamera webcam;
//
//    @Override
//    public void runOpMode() {
//        // Initialize webcam
//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//
//        // Set the pipeline
//
//        AutoAlignSamplePipeline pipeline = new AutoAlignSamplePipeline();
//        webcam.setPipeline(pipeline);
//
//        double currentAngle = pipeline.getSampleAngle();
//        telemetry.addData("Angle", currentAngle);
//
//        // Start streaming
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addData("Camera Error", errorCode);
//            }
//        });
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            telemetry.addData("FPS", webcam.getFps());
//            telemetry.update();
//        }
//
//        // Stop camera when done
//        webcam.stopStreaming();
//    }
//}

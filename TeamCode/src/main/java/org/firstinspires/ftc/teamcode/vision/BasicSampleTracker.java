package org.firstinspires.ftc.teamcode.vision;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.rrfiles.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@TeleOp
public class BasicSampleTracker extends OpMode {

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution

    samplePickUp tracker = new samplePickUp();
    ElapsedTime delay = new ElapsedTime();

    Pose2d startPose = new Pose2d(-38, -61, Math.toRadians(0));
    MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;

    Servo intakeArmLeft;
    Servo intakeArmRight;

    Servo intakeDiffyLeft;
    Servo intakeDiffyRight;

    Servo linkage1;
    Servo linkage2;

    Servo intakeClaw;

    @Override
    public void init(){
        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        leftBack = hardwareMap.get(DcMotorEx.class, "LB");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");
        rightBack = hardwareMap.get(DcMotorEx.class, "RB");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeArmLeft = hardwareMap.get(Servo.class, "intakeArmLeft");
        intakeArmRight = hardwareMap.get(Servo.class, "intakeArmRight");

        intakeDiffyLeft = hardwareMap.get(Servo.class, "intakeDiffyLeft");
        intakeDiffyRight = hardwareMap.get(Servo.class, "intakeDiffyRight");

        linkage1 = hardwareMap.get(Servo.class, "linkage1");
        linkage2 = hardwareMap.get(Servo.class, "linkage2");

        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");

        Robot.intake.transfer(linkage1, linkage2, intakeArmLeft, intakeArmRight, intakeDiffyLeft, intakeDiffyRight, intakeClaw);

    }

    public void loop(){
        double angle = tracker.getCurrAngle();
        String side = tracker.getSide();
        String zone = tracker.getZone();

//        if (gamepad1.right_trigger > 0.2) {
//            intakeArmLeft.setPosition(0.39);
//            intakeArmRight.setPosition(0.62);
//            intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
//        }

        if (gamepad1.a) {
            alignAndIntake(delay);
        }

        if (gamepad1.b) {
            tracker.alignToSampleHorizontal(leftFront, leftBack, rightFront, rightBack, 0.265, drive);
        }

        if (gamepad1.x) {
            tracker.alignToSampleVertical(leftFront, leftBack, rightFront, rightBack, 0.15, drive);
        }

        telemetry.addData("Sample Angle", angle);
        telemetry.addData("Zone", zone);
        telemetry.addData("Side", side);
        telemetry.update();
    }

    public void stop(){
        telemetry.addData("Status", "OpMode Stopped");
        telemetry.update();
        controlHubCam.stopStreaming();
        controlHubCam.closeCameraDevice();
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
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.SIDEWAYS_RIGHT);
        controlHubCam.setPipeline(tracker);
    }

    private void alignAndIntake(ElapsedTime delay) {
        intakeArmLeft.setPosition(0.39);
        intakeArmRight.setPosition(0.62);
        delay.reset();
        while (delay.milliseconds() < 700) {

        }
        intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
        tracker.alignToSampleHorizontal(leftFront, leftBack, rightFront, rightBack, 0.265, drive);
        tracker.alignToSampleVertical(leftFront, leftBack, rightFront, rightBack, 0.15, drive);
        tracker.clawAlign(intakeDiffyLeft, intakeDiffyRight);
        delay.reset();
        while (delay.milliseconds() < 700) {

        }
        intakeArmLeft.setPosition(Robot.INTAKE_ARM_LEFT_EXTEND_TELEOP);
        intakeArmRight.setPosition(Robot.INTAKE_ARM_RIGHT_EXTEND_TELEOP);
        delay.reset();
        while (delay.milliseconds() < 500) {

        }
        intakeClaw.setPosition(Robot.INTAKE_CLAW_CLOSE);
        delay.reset();
        while (delay.milliseconds() < 500) {

        }
        Robot.intake.transfer(linkage1, linkage2, intakeArmLeft, intakeArmRight, intakeDiffyLeft, intakeDiffyRight, intakeClaw);
    }

}
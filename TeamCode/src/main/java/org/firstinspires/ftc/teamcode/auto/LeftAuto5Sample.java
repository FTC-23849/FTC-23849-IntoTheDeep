package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.rrfiles.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.vision.samplePickUp;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class LeftAuto5Sample extends LinearOpMode {

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution
    String alliance = null;

    samplePickUp tracker = new samplePickUp();
    ElapsedTime delay = new ElapsedTime();

    @Override
    public void runOpMode() {
        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        DcMotorEx leftFront;
        DcMotorEx leftBack;
        DcMotorEx rightFront;
        DcMotorEx rightBack;

        Servo linkage1;
        Servo linkage2;

        Servo outtakeClaw;
        Servo outtakeWrist;
        Servo outtakeArm;
        Servo outtakeArm2;
//
        DcMotorEx slideMotor_back;
        DcMotorEx slideMotor_front;
        DcMotorEx slideMotor_up;
        Servo intakeDiffyLeft;
        Servo intakeDiffyRight;
        Servo intakeArmLeft;
        Servo intakeArmRight;
        Servo intakeClaw;

        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        leftBack = hardwareMap.get(DcMotorEx.class, "LB");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");
        rightBack = hardwareMap.get(DcMotorEx.class, "RB");

        linkage1 = hardwareMap.get(Servo.class, "linkage1");
        linkage2 = hardwareMap.get(Servo.class, "linkage2");

        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");
        outtakeWrist = hardwareMap.get(Servo.class, "outtakeWrist");
        outtakeArm = hardwareMap.get(Servo.class, "outtakeArm");
        outtakeArm2 = hardwareMap.get(Servo.class,"outtakeArm2");

//
        slideMotor_back = hardwareMap.get(DcMotorEx.class, "slideMotor_left");
        slideMotor_front = hardwareMap.get(DcMotorEx.class, "slideMotor_right");
        slideMotor_up = hardwareMap.get(DcMotorEx.class, "slideMotor_up");

        slideMotor_back.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); //likely not needed, but sets encoders to 0.
        slideMotor_back.setTargetPosition(0);
        slideMotor_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor_back.setPower(1.0);

        slideMotor_up.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); //likely not needed, but sets encoders to 0.
        slideMotor_up.setTargetPosition(0);
        slideMotor_up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor_up.setPower(1.0);

        slideMotor_front.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); //likely not needed, but sets encoders to 0.
        slideMotor_front.setTargetPosition(0);
        slideMotor_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor_front.setPower(1.0);
        slideMotor_front.setDirection(DcMotorEx.Direction.REVERSE);
        intakeArmLeft = hardwareMap.get(Servo.class, "intakeArmLeft");
        intakeArmRight = hardwareMap.get(Servo.class, "intakeArmRight");
        intakeDiffyLeft = hardwareMap.get(Servo.class, "intakeDiffyLeft");
        intakeDiffyRight = hardwareMap.get(Servo.class, "intakeDiffyRight");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");

        Robot.intake.transfer(linkage1, linkage2, intakeArmLeft, intakeArmRight, intakeDiffyLeft, intakeDiffyRight, intakeClaw);
        Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeArm2, outtakeWrist);
        Robot.outtake.closeClaw(outtakeClaw);
        intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
//
//        ElapsedTime outtakeTimer = new ElapsedTime();

        // Create Roadrunner Trajectories
        Pose2d startPose = new Pose2d(-38, -61, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Pose2d currentPose = drive.pose;

        TrajectoryActionBuilder scorePreload = drive.actionBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.toRadians(200));

        TrajectoryActionBuilder collectSample1 = drive.actionBuilder(new Pose2d(-57, -57, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(-55.5, -48.5, Math.toRadians(76)), Math.toRadians(250));

        TrajectoryActionBuilder scoreSample1 = drive.actionBuilder(new Pose2d(-57, -57, Math.toRadians(46)))
                .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), Math.toRadians(90));

        TrajectoryActionBuilder collectSample2 = drive.actionBuilder(new Pose2d(-52, -52, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(-57.5, -50, Math.toRadians(98)), Math.toRadians(90));

        TrajectoryActionBuilder scoreSample2 = drive.actionBuilder(new Pose2d(-57, -50, Math.toRadians(100)))
                .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), Math.toRadians(90));

        TrajectoryActionBuilder collectSample3 = drive.actionBuilder(new Pose2d(-52, -52, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(-50, -43, Math.toRadians(140)), Math.toRadians(90));

        TrajectoryActionBuilder scoreSample3 = drive.actionBuilder(new Pose2d(-50, -43, Math.toRadians(140)))
                .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), Math.toRadians(90));

        TrajectoryActionBuilder collectSub1 = drive.actionBuilder(new Pose2d(-52, -52, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(-22, -5, Math.toRadians(0)), Math.toRadians(-10));

        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(-52, -52, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(-17, -5, Math.toRadians(180)), Math.toRadians(-10));

        Robot.intake.transfer(linkage1, linkage2, intakeArmLeft, intakeArmRight, intakeDiffyLeft, intakeDiffyRight, intakeClaw);
        Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeArm2, outtakeWrist);
        Robot.outtake.closeClaw(outtakeClaw);
        intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);

        waitForStart();

        if (gamepad2.x) {
            alliance = "blue";
            telemetry.clearAll();
            telemetry.addData("Alliance", alliance);
            telemetry.update();
        } else if (gamepad2.b){
            alliance = "red";
            telemetry.clearAll();
            telemetry.addData("Alliance", alliance);
            telemetry.update();
        }

        telemetry.update();

        while (opModeIsActive()) {
            if (isStopRequested()) {
                telemetry.addData("Status", "OpMode Stopped");
                telemetry.update();
                controlHubCam.stopStreaming();
                controlHubCam.closeCameraDevice();
                return;
            }

            sleep(4);

            // create variables for camera stream

            double angle = tracker.getCurrAngle();
            String side = tracker.getSide();
            String zone = tracker.getZone();

            Actions.runBlocking(new ParallelAction(
                    new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 3000, 1.0),
                    scorePreload.build()
            ));

            Robot.outtake.scoreSample(outtakeArm, outtakeArm2, outtakeWrist);
            sleep(500);
            Robot.outtake.openClaw(outtakeClaw);
            sleep(200);

            Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeArm2, outtakeWrist);
            Actions.runBlocking(new ParallelAction(
                    new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 0, 1.0),
                    collectSample1.build()
            ));

            intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
            linkage1.setPosition(0.22/*0.15*/);
            linkage2.setPosition(0.78/*0.83*/);
            intakeDiffyLeft.setPosition(Robot.INTAKE_LEFT_DIFFY_PICK_UP);
            intakeDiffyRight.setPosition(Robot.INTAKE_RIGHT_DIFFY_PICK_UP);
            sleep(500);
            intakeArmLeft.setPosition(Robot.INTAKE_ARM_LEFT_EXTEND);
            intakeArmRight.setPosition(Robot.INTAKE_ARM_RIGHT_EXTEND);
            sleep(700);
            intakeClaw.setPosition(Robot.INTAKE_CLAW_CLOSE);
            sleep(650);
            Robot.intake.transfer(linkage1, linkage2, intakeArmLeft, intakeArmRight, intakeDiffyLeft, intakeDiffyRight, intakeClaw);
            linkage1.setPosition(0.04);
            linkage2.setPosition(0.97);
            sleep(800);
            Robot.outtake.closeClaw(outtakeClaw);
            sleep(100);
            intakeClaw.setPosition(Robot.OPEN_CLAW);

            Actions.runBlocking(new ParallelAction(
                    new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 3000, 1.0),
                    scoreSample1.build()
            ));

            Robot.outtake.scoreSample(outtakeArm, outtakeArm2, outtakeWrist);
            sleep(500);
            Robot.outtake.openClaw(outtakeClaw);
            sleep(200);

            Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeArm2, outtakeWrist);
            Actions.runBlocking(new ParallelAction(
                    new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 0, 1.0),
                    collectSample2.build()
            ));

            intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
            linkage1.setPosition(0.24/*0.16*/);
            linkage2.setPosition(0.76/*0.84*/);
            intakeDiffyLeft.setPosition(Robot.INTAKE_LEFT_DIFFY_PICK_UP);
            intakeDiffyRight.setPosition(Robot.INTAKE_RIGHT_DIFFY_PICK_UP);
            sleep(500);
            intakeArmLeft.setPosition(Robot.INTAKE_ARM_LEFT_EXTEND);
            intakeArmRight.setPosition(Robot.INTAKE_ARM_RIGHT_EXTEND);
            sleep(600);
            intakeClaw.setPosition(Robot.INTAKE_CLAW_CLOSE);
            sleep(650);
            Robot.intake.transfer(linkage1, linkage2, intakeArmLeft, intakeArmRight, intakeDiffyLeft, intakeDiffyRight, intakeClaw);
            linkage1.setPosition(0.04);
            linkage2.setPosition(0.97);
            sleep(800);
            Robot.outtake.closeClaw(outtakeClaw);
            sleep(100);
            intakeClaw.setPosition(Robot.OPEN_CLAW);

            Actions.runBlocking(new ParallelAction(
                    new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 3000, 1.0),
                    scoreSample2.build()
            ));

            Robot.outtake.scoreSample(outtakeArm, outtakeArm2, outtakeWrist);
            sleep(500);
            Robot.outtake.openClaw(outtakeClaw);
            sleep(200);

            Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeArm2, outtakeWrist);
            Actions.runBlocking(new ParallelAction(
                    new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 0, 1.0),
                    collectSample3.build()
            ));

            intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
            linkage1.setPosition(0.20/*0.15*/);
            linkage2.setPosition(0.80/*0.85*/);
            intakeDiffyLeft.setPosition(Robot.INTAKE_LEFT_DIFFY_PICK_UP_WALL_SAMPLE);
            intakeDiffyRight.setPosition(Robot.INTAKE_RIGHT_DIFFY_PICK_UP_WALL_SAMPLE);
            sleep(500);
            intakeArmLeft.setPosition(Robot.INTAKE_ARM_LEFT_EXTEND);
            intakeArmRight.setPosition(Robot.INTAKE_ARM_RIGHT_EXTEND);
            sleep(700);
            intakeClaw.setPosition(Robot.INTAKE_CLAW_CLOSE);
            sleep(650);
            Robot.intake.transfer(linkage1, linkage2, intakeArmLeft, intakeArmRight, intakeDiffyLeft, intakeDiffyRight, intakeClaw);
            sleep(800);
            Robot.outtake.closeClaw(outtakeClaw);
            sleep(100);
            intakeClaw.setPosition(Robot.OPEN_CLAW);

            Actions.runBlocking(new ParallelAction(
                    new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 3000, 1.0),
                    scoreSample3.build()
            ));

            Robot.outtake.scoreSample(outtakeArm, outtakeArm2, outtakeWrist);
            sleep(500);
            Robot.outtake.openClaw(outtakeClaw);
            sleep(200);

            Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeArm2, outtakeWrist);
            Actions.runBlocking(new ParallelAction(
                    new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 0, 1.0),
                    collectSub1.build()
            ));

            drive.updatePoseEstimate();
            drive.localizer.update();
            telemetry.addData("Vector2d", currentPose.position);
            telemetry.addData("Heading", currentPose.heading);
            telemetry.update();

            alignAndIntake(delay, leftFront, leftBack, rightFront, rightBack, linkage1, linkage2, intakeArmLeft, intakeArmRight, intakeDiffyLeft, intakeDiffyRight, intakeClaw, drive);

            drive.updatePoseEstimate();
            drive.localizer.update();

            sleep(800);
            Robot.outtake.closeClaw(outtakeClaw);
            sleep(100);
            intakeClaw.setPosition(Robot.OPEN_CLAW);

            TrajectoryActionBuilder scoreSub1 = drive.actionBuilder(drive.pose)
                    .setTangent(85)
                    .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.toRadians(260));

            Actions.runBlocking(new ParallelAction(
                    new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 3000, 1.0),
                    scoreSub1.build()
            ));

            Robot.outtake.scoreSample(outtakeArm, outtakeArm2, outtakeWrist);
            sleep(500);
            Robot.outtake.openClaw(outtakeClaw);
            sleep(200);

            Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeArm2, outtakeWrist);
            Actions.runBlocking(new ParallelAction(
                    new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 850, 1.0),
                    park.build()
            ));

            sleep(1000000000);
        }

    }

    private void alignAndIntake(ElapsedTime delay, DcMotorEx leftFront, DcMotorEx leftBack,
                                DcMotorEx rightFront, DcMotorEx rightBack, Servo linkage1, Servo linkage2,
                                Servo intakeArmLeft, Servo intakeArmRight, Servo intakeDiffyLeft,
                                Servo intakeDiffyRight, Servo intakeClaw, MecanumDrive drive) {
        intakeArmLeft.setPosition(0.35); //decrease to lower arm detection position
        intakeArmRight.setPosition(0.66);
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

    private static void intakeSample(Servo intakeClaw, Servo intakeArmLeft, Servo intakeArmRight){
        intakeArmLeft.setPosition(Robot.INTAKE_ARM_LEFT_EXTEND);
        intakeArmRight.setPosition(Robot.INTAKE_ARM_RIGHT_EXTEND);
        intakeClaw.setPosition(Robot.INTAKE_CLAW_CLOSE);
    }


    public class outtakeSlides {

        DcMotorEx slideMotor_back;
        DcMotorEx slideMotor_front;

        public void scoreSample(){
            slideMotor_back.setTargetPosition(1000);
            slideMotor_front.setTargetPosition(1000);
        }

    }

    public class setOuttakeSlidesPatient implements Action{

        DcMotorEx slideMotor_back;
        DcMotorEx slideMotor_front;
        DcMotorEx slideMotor_up;
        int ticks;
        double p;

        public setOuttakeSlidesPatient(DcMotorEx back, DcMotorEx front, DcMotorEx up, int ticks, double power){
            this.slideMotor_back = back;
            this.slideMotor_front = front;
            this.slideMotor_up = up;
            this.ticks = ticks;
            this.p = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            slideMotor_back.setTargetPosition(-ticks);
            slideMotor_front.setTargetPosition(-ticks);
            slideMotor_up.setTargetPosition(-ticks);

            slideMotor_front.setPower(p);
            slideMotor_back.setPower(-p);
            slideMotor_up.setPower(-p);

            return slideMotor_front.isBusy() && slideMotor_back.isBusy() && slideMotor_up.isBusy();
        }
    }

    public class setOuttakeSlidesNoDetection implements Action{

        DcMotorEx slideMotor_back;
        DcMotorEx slideMotor_front;
        int ticks;

        public setOuttakeSlidesNoDetection(DcMotorEx back, DcMotorEx front, int ticks){
            this.slideMotor_back = back;
            this.slideMotor_front = front;
            this.ticks = ticks;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            slideMotor_back.setTargetPosition(ticks);
            slideMotor_front.setTargetPosition(-ticks);

            return false;
        }
    }

}
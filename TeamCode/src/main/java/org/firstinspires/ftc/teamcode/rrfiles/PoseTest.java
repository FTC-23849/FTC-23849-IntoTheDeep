package org.firstinspires.ftc.teamcode.rrfiles;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;


@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class PoseTest extends LinearOpMode {

    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() {

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

        DcMotorEx leftFront;
        DcMotorEx leftBack;
        DcMotorEx rightFront;
        DcMotorEx rightBack;

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
        drive.updatePoseEstimate();
        drive.localizer.update();

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

        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(-52, -52, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(-17, -5, Math.toRadians(180)), Math.toRadians(-10));

        Robot.intake.transfer(linkage1, linkage2, intakeArmLeft, intakeArmRight, intakeDiffyLeft, intakeDiffyRight, intakeClaw);
        Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeArm2, outtakeWrist);
        Robot.outtake.closeClaw(outtakeClaw);
        intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);

        waitForStart();

        if (isStopRequested()) return;

        sleep(4);

//        Actions.runBlocking(new ParallelAction(
//                (drive.actionBuilder(startPose)
//                        .splineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(45)), Math.toRadians(200))
//                        .build())
//        ));

        telemetry.addData("initial Vector2d", drive.pose.position);
        telemetry.addData("initial heading", drive.pose.heading);
        telemetry.update();

        sleep(10000);

        Actions.runBlocking(scorePreload.build());

        drive.updatePoseEstimate();
        drive.localizer.update();

        telemetry.addData("TargetEstimate", "new Pose2d(-53, -53, Math.toRadians(45))");
        telemetry.addData("Vector2d", drive.pose.position);
        telemetry.addData("Heading", drive.pose.heading);
        telemetry.update();

        sleep(10000);

        time.reset();
        while (time.milliseconds() < 700) {
            drive.updatePoseEstimate();
            drive.localizer.update();
            leftFront.setPower(0.3);
            leftBack.setPower(0.3);
            rightFront.setPower(0.3);
            rightBack.setPower(0.3);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        drive.updatePoseEstimate();
        drive.localizer.update();
        telemetry.addData("Vector2d", drive.pose.position);
        telemetry.addData("Heading", drive.pose.heading);
        telemetry.update();

        sleep(10000);

        TrajectoryActionBuilder backtobasket = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-53, -53));

        Actions.runBlocking(backtobasket.build());

        drive.updatePoseEstimate();
        drive.localizer.update();

        telemetry.addData("AfterStrafeTo", "new Pose2d(-53, -53, Math.toRadians(45))");
        telemetry.addData("Vector2d", drive.pose.position);
        telemetry.addData("Heading", drive.pose.heading);
        telemetry.update();

        sleep(10000);

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

            return slideMotor_front.isBusy() || slideMotor_back.isBusy() || slideMotor_up.isBusy();
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
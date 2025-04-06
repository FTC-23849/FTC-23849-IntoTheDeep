package org.firstinspires.ftc.teamcode.archive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;


//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class LeftAutoBackup extends LinearOpMode {

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

        TrajectoryActionBuilder scorePreload = drive.actionBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.toRadians(200));

        TrajectoryActionBuilder collectSample1 = drive.actionBuilder(new Pose2d(-57, -57, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(-55.5, -48.5, Math.toRadians(76)), Math.toRadians(250));

        TrajectoryActionBuilder scoreSample1 = drive.actionBuilder(new Pose2d(-57, -57, Math.toRadians(46)))
                .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), Math.toRadians(90));

        TrajectoryActionBuilder collectSample2 = drive.actionBuilder(new Pose2d(-52, -52, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(98)), Math.toRadians(90));

        TrajectoryActionBuilder scoreSample2 = drive.actionBuilder(new Pose2d(-57.5, -50, Math.toRadians(100)))
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
                new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 850, 1.0),
                park.build()
        ));

        sleep(5000);


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
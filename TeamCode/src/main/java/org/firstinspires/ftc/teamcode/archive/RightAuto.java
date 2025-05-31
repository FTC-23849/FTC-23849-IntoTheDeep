package org.firstinspires.ftc.teamcode.archive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.rrfiles.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;


@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class RightAuto extends LinearOpMode {

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

        slideMotor_back.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); //likely not needed, but sets encoders to 0.
        slideMotor_back.setTargetPosition(0);
        slideMotor_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor_back.setPower(1.0);

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

        Pose2d startPose = new Pose2d(9, -61, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        drive.localizer.update();
        Pose2d currentPose = drive.pose;


        TrajectoryActionBuilder scorePreload = drive.actionBuilder(startPose)
                .lineToY(-33);

        TrajectoryActionBuilder pushSamples = drive.actionBuilder(new Pose2d(9, -33, Math.toRadians(270)))
                .lineToY(-38)
                .splineToLinearHeading(new Pose2d(35, -40, Math.toRadians(90)), Math.toRadians(80))
                .splineToConstantHeading(new Vector2d(43, -10), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(55, -55), Math.toRadians(30))
                .setTangent(90)
                .splineToConstantHeading(new Vector2d(55, -10), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60, -55), Math.toRadians(10));

        TrajectoryActionBuilder collectSpecimen1 = drive.actionBuilder(new Pose2d(60, -55, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(47, -52), Math.toRadians(180))
                .setTangent(1.6)
                .lineToY(-57);

        TrajectoryActionBuilder scoreSpecimen1 = drive.actionBuilder(new Pose2d(47, -57, Math.toRadians(90)))
                .setTangent(15)
                .splineToLinearHeading(new Pose2d(0, -32, Math.toRadians(270)), Math.toRadians(90));

        TrajectoryActionBuilder collectSpecimen2 = drive.actionBuilder(new Pose2d(0, -33, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(47, -52, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(1.6)
                .lineToY(-57);

        TrajectoryActionBuilder scoreSpecimen2 = drive.actionBuilder(new Pose2d(47, -58, Math.toRadians(90)))
                .setTangent(15)
                .splineToLinearHeading(new Pose2d(-5, -31, Math.toRadians(270)), Math.toRadians(90));

        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(-5, -33, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(50, -55, Math.toRadians(90)), Math.toRadians(90));

        Robot.intake.transfer(linkage1, linkage2, intakeArmLeft, intakeArmRight, intakeDiffyLeft, intakeDiffyRight, intakeClaw);
        Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeArm2, outtakeWrist);
        Robot.outtake.closeClaw(outtakeClaw);
        intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
        intakeDiffyLeft.setPosition(0.62);
        intakeDiffyRight.setPosition(0.39);

        waitForStart();

        if (isStopRequested()) return;

        sleep(4);

        Actions.runBlocking(new ParallelAction(
                new intakeOpenClose(linkage1, linkage2),
                new scoreSpecimen(outtakeArm, outtakeArm2, outtakeWrist, outtakeClaw),
                new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, 1450, 1.0),
                scorePreload.build()
        ));

        Actions.runBlocking(new SequentialAction(
                new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, 950, 1.0),
                new ParallelAction(
                        new openClaw(outtakeClaw),
                        new restArm(outtakeArm),
                        new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, 0, 1.0),
                        pushSamples.build()
                )
        ));

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                    new receiveSpecimen(outtakeClaw, outtakeWrist, outtakeArm, outtakeArm2),
                    collectSpecimen1.build()),
                new closeClaw(outtakeClaw),
                new ParallelAction(
                        new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, 1600, 1.0),
                        scoreSpecimen1.build()
                )
        ));

        Actions.runBlocking(new SequentialAction(
                new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, 950, 1.0),
                new ParallelAction(
                        new openClaw(outtakeClaw),
                        new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, 0, 1.0),
                        collectSpecimen2.build()
                )
        ));

        Actions.runBlocking(new SequentialAction(
                new closeClaw(outtakeClaw),
                new ParallelAction(
                        new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, 1600, 1.0),
                        scoreSpecimen2.build()
                )
        ));

        Actions.runBlocking(new SequentialAction(
                new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, 950, 1.0),
                new ParallelAction(
                        new openClaw(outtakeClaw),
                        new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, 0, 1.0)
                        //collectSpecimen3.build()
                )
        ));

        Actions.runBlocking(new ParallelAction(
                new restArm(outtakeArm),
                new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, 0, 1.0),
                park.build()
        ));

        sleep(1000);


    }


    public class setOuttakeSlidesPatient implements Action{

        DcMotorEx slideMotor_back;
        DcMotorEx slideMotor_front;
        int ticks;
        double p;

        public setOuttakeSlidesPatient(DcMotorEx back, DcMotorEx front, int ticks, double power){
            this.slideMotor_back = back;
            this.slideMotor_front = front;
            this.ticks = ticks;
            this.p = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            slideMotor_back.setTargetPosition(-ticks);
            slideMotor_front.setTargetPosition(-ticks);

            slideMotor_front.setPower(p);
            slideMotor_back.setPower(-p);

            return slideMotor_front.isBusy() || slideMotor_back.isBusy();
        }
    }

    public class openClaw implements Action{
        Servo outtakeClaw;

        public openClaw(Servo outtakeClaw){
            this.outtakeClaw = outtakeClaw;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Robot.outtake.openClaw(outtakeClaw);

            return false;
        }
    }

    public class closeClaw implements Action{
        Servo outtakeClaw;

        public closeClaw(Servo outtakeClaw){
            this.outtakeClaw = outtakeClaw;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Robot.outtake.closeClaw(outtakeClaw);

            return false;
        }
    }

    public class scoreSpecimen implements Action{

        Servo outtakeArm;
        Servo outtakeArm2;
        Servo outtakeWrist;
        Servo outtakeClaw;

        public scoreSpecimen(Servo outtakeArm, Servo outtakeArm2, Servo outtakeWrist, Servo outtakeClaw){
            this.outtakeArm = outtakeArm;
            this.outtakeArm2 = outtakeArm2;
            this.outtakeWrist = outtakeWrist;
            this.outtakeClaw = outtakeClaw;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Robot.outtake.scoreSpecimen(outtakeArm, outtakeArm2, outtakeWrist, outtakeClaw);

            return false;
        }
    }

    public class receiveSpecimen implements Action{

        Servo outtakeArm;
        Servo outtakeArm2;
        Servo outtakeWrist;
        Servo outtakeClaw;

        public receiveSpecimen(Servo outtakeClaw, Servo outtakeWrist, Servo outtakeArm, Servo outtakeArm2){
            this.outtakeArm = outtakeArm;
            this.outtakeArm2 = outtakeArm2;
            this.outtakeWrist = outtakeWrist;
            this.outtakeClaw = outtakeClaw;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Robot.outtake.specimenReceivePosition(outtakeClaw, outtakeWrist, outtakeArm, outtakeArm2);

            return false;
        }
    }

    public class intakeOpenClose implements Action{

        Servo linkage1;
        Servo linkage2;

        public intakeOpenClose(Servo linkage1, Servo linkage2){
            this.linkage1 = linkage1;
            this.linkage2 = linkage2;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            linkage1.setPosition(0.1);
            linkage2.setPosition(0.9);

            linkage1.setPosition(0.0);
            linkage2.setPosition(1.0);

            return false;
        }
    }

    public class restArm implements Action{

        Servo outtakeArm;

        public restArm(Servo outtakeArm){
            this.outtakeArm = outtakeArm;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            outtakeArm.setPosition(0.75);
            return false;
        }
    }


}
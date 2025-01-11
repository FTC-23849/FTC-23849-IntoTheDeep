package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class RightAutoExtendingArmBackUp extends LinearOpMode {

    @Override
    public void runOpMode() {

        Servo linkage1;
        Servo linkage2;

        Servo outtakeClaw;
        Servo outtakeWrist;
        Servo outtakeArm;
        Servo outtakeArm2;
        Servo outtakeSupport;
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
        outtakeSupport = hardwareMap.get(Servo.class,"outtakeSupport");

        Robot.intake.transfer(linkage1, linkage2, intakeArmLeft, intakeArmRight, intakeDiffyLeft, intakeDiffyRight, intakeClaw);
        Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeArm2, outtakeWrist);
        Robot.outtake.closeClaw(outtakeClaw);
        intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
        intakeDiffyLeft.setPosition(0.72);
        intakeDiffyRight.setPosition(0.30);
//
//        ElapsedTime outtakeTimer = new ElapsedTime();

        // Create Roadrunner Trajectories

        Pose2d startPose = new Pose2d(9, -61, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        TrajectoryActionBuilder scorePreload = drive.actionBuilder(startPose)
                .lineToY(-31);

        TrajectoryActionBuilder pushSamples = drive.actionBuilder(new Pose2d(9, -31, Math.toRadians(270)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(31, -40, Math.toRadians(90)), Math.toRadians(10), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70))
                .splineToConstantHeading(new Vector2d(50, -10), Math.toRadians(10), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70))
                .setTangent(Math.toRadians(270))
                .lineToY(-53, new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100, 100))
                .setTangent(Math.toRadians(270))
                .lineToY(-10, new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100, 100))
                .setTangent(Math.toRadians(0))
                .lineToX(59, new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100, 100))
                .setTangent(Math.toRadians(270))
                .lineToY(-53, new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100, 100));

        TrajectoryActionBuilder collectSpecimen1 = drive.actionBuilder(new Pose2d(60, -53, Math.toRadians(90)))
                .lineToY(-62, new TranslationalVelConstraint(15), new ProfileAccelConstraint(-15, 15));

        TrajectoryActionBuilder scoreSpecimen1 = drive.actionBuilder(new Pose2d(47, -62, Math.toRadians(90)))
                .setTangent(15)
                .splineToConstantHeading(new Vector2d(3, -28), Math.toRadians(90), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-30, 50));

        TrajectoryActionBuilder collectSpecimen2 = drive.actionBuilder(new Pose2d(0, -28, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .lineToY(-32)
                .splineToConstantHeading(new Vector2d(47, -50), Math.toRadians(0), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-40, 40))
                .setTangent(Math.toRadians(90))
                .lineToY(-60);

        TrajectoryActionBuilder scoreSpecimen2 = drive.actionBuilder(new Pose2d(47, -60, Math.toRadians(90)))
                .setTangent(15)
                .splineToConstantHeading(new Vector2d(-1, -27), Math.toRadians(90), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-30, 50));

        TrajectoryActionBuilder collectSpecimen3 = drive.actionBuilder(new Pose2d(-2, -28, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .lineToY(-32)
                .splineToConstantHeading(new Vector2d(47, -52), Math.toRadians(0), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-40, 40))
                .setTangent(Math.toRadians(90))
                .lineToY(-61);

        TrajectoryActionBuilder scoreSpecimen3 = drive.actionBuilder(new Pose2d(47, -59, Math.toRadians(90)))
                .setTangent(15)
                .splineToConstantHeading(new Vector2d(-3, -27), Math.toRadians(90), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-30, 50));

        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(-5, -29, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(50, -55, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70));

        Robot.intake.transfer(linkage1, linkage2, intakeArmLeft, intakeArmRight, intakeDiffyLeft, intakeDiffyRight, intakeClaw);
        Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeArm2, outtakeWrist);
        Robot.outtake.closeClaw(outtakeClaw);
        intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
        intakeDiffyLeft.setPosition(0.72);
        intakeDiffyRight.setPosition(0.30);

        waitForStart();

        if (isStopRequested()) return;

        sleep(4);

        Actions.runBlocking(new ParallelAction(
                new intakeOpenClose(linkage1, linkage2),
                new scoreSpecimenPlain(outtakeWrist, outtakeArm, outtakeArm2),
                new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 1450, 1.0),
                scorePreload.build()
        ));

        Actions.runBlocking(new SequentialAction(
                new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 950, 1.0),
                new ParallelAction(
                        new openClaw(outtakeClaw),
                        new restArm(outtakeClaw, outtakeArm, outtakeArm2, outtakeWrist),
                        new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 0, 1.0),
                        pushSamples.build()
                )
        ));

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        new receiveSpecimen(outtakeClaw, outtakeWrist, outtakeArm, outtakeArm2),
                        collectSpecimen1.build()),
                new closeClaw(outtakeClaw),
                new ParallelAction(
                        new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 1550, 1.0),
                        new scoreSpecimen(outtakeArm, outtakeArm2, outtakeWrist, outtakeClaw),
                        scoreSpecimen1.build()
                ),
                new raiseSupport(outtakeSupport)
        ));

        Actions.runBlocking(new SequentialAction(
                new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 760, 1.0),
                new dropSupport(outtakeSupport),
                new ParallelAction(
                        new openClaw(outtakeClaw),
                        new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 0, 1.0),
                        new receiveSpecimen(outtakeClaw, outtakeWrist, outtakeArm, outtakeArm2),
                        collectSpecimen2.build()
                )
        ));

        Actions.runBlocking(new SequentialAction(
                new closeClaw(outtakeClaw),
                new ParallelAction(
                        new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 1550, 1.0),
                        new scoreSpecimen(outtakeArm, outtakeArm2, outtakeWrist, outtakeClaw),
                        scoreSpecimen2.build()
                ),
                new raiseSupport(outtakeSupport)
        ));

        Actions.runBlocking(new SequentialAction(
                new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 800, 1.0),
                new dropSupport(outtakeSupport),
                new ParallelAction(
                        new openClaw(outtakeClaw),
                        new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 10, 1.0),
                        new receiveSpecimen(outtakeClaw, outtakeWrist, outtakeArm, outtakeArm2),
                        collectSpecimen3.build()
                )
        ));

        Actions.runBlocking(new SequentialAction(
                new closeClaw(outtakeClaw),
                new ParallelAction(
                        new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 1550, 1.0),
                        new scoreSpecimen(outtakeArm, outtakeArm2, outtakeWrist, outtakeClaw),
                        scoreSpecimen3.build()
                ),
                new raiseSupport(outtakeSupport)
        ));

        Actions.runBlocking(new SequentialAction(
                new dropSupport(outtakeSupport),
                new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 760, 1.0),
                new ParallelAction(
                        new openClaw(outtakeClaw),
                        new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 0, 1.0)
                )
        ));

        Actions.runBlocking(new ParallelAction(
                new restArm(outtakeClaw, outtakeArm, outtakeArm2, outtakeWrist),
                new setOuttakeSlidesPatient(slideMotor_back, slideMotor_front, slideMotor_up, 0, 1.0),
                park.build()
        ));

        sleep(1000);


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
            slideMotor_up.setPower(p);

            return slideMotor_front.isBusy() || slideMotor_back.isBusy() || slideMotor_up.isBusy();
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

    public class scoreSpecimenPlain implements Action{

        Servo outtakeWrist;
        Servo outtakeArm;
        Servo outtakeArm2;

        public scoreSpecimenPlain(Servo outtakeWrist, Servo outtakeArm, Servo outtakeArm2){
            this.outtakeWrist = outtakeWrist;
            this.outtakeArm = outtakeArm;
            this.outtakeArm2 = outtakeArm2;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Robot.outtake.specimenReceivePositionPlain(outtakeWrist, outtakeArm, outtakeArm2);

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

    public class raiseSupport implements Action{

        Servo outtakeSupport;

        public raiseSupport(Servo outtakeSupport){
            this.outtakeSupport = outtakeSupport;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Robot.outtake.extendSupport(outtakeSupport);

            return false;
        }
    }

    public class dropSupport implements Action{

        Servo outtakeSupport;

        public dropSupport(Servo outtakeSupport){
            this.outtakeSupport = outtakeSupport;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Robot.outtake.retractSupport(outtakeSupport);

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

        Servo outtakeClaw;
        Servo outtakeArm;
        Servo outtakeArm2;
        Servo outtakeWrist;

        public restArm(Servo outtakeClaw, Servo outtakeArm, Servo outtakeArm2, Servo outtakeWrist){
            this.outtakeClaw = outtakeClaw;
            this.outtakeArm = outtakeArm;
            this.outtakeArm2 = outtakeArm2;
            this.outtakeWrist = outtakeWrist;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeArm2, outtakeWrist);
            return false;
        }
    }


}
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class LeftAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        Servo linkage1;
        Servo linkage2;
        Servo bucket;
        CRServo intakeRollers;

        Servo outtakeClaw;
        Servo outtakeWrist;
        Servo outtakeArm;

        DcMotorEx slideMotor_left;
        DcMotorEx slideMotor_right;

        linkage1 = hardwareMap.get(Servo.class, "linkage1");
        linkage2 = hardwareMap.get(Servo.class, "linkage2");
        bucket = hardwareMap.get(Servo.class, "bucket");
        intakeRollers = hardwareMap.get(CRServo.class, "intakeRollers");

        outtakeClaw = hardwareMap.get(Servo.class,"outtakeClaw");
        outtakeWrist = hardwareMap.get(Servo.class,"outtakeWrist");
        outtakeArm = hardwareMap.get(Servo.class,"outtakeArm");

        slideMotor_left = hardwareMap.get(DcMotorEx.class,"slideMotor_left");
        slideMotor_right = hardwareMap.get(DcMotorEx.class,"slideMotor_right");
        slideMotor_right.setDirection(DcMotorEx.Direction.REVERSE);

        Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeWrist);
        Robot.intake.transfer(linkage1, linkage2, bucket);

        ElapsedTime outtakeTimer = new ElapsedTime();

        // Create Roadrunner Trajectories

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-38, -61, Math.toRadians(0));

        TrajectorySequence scoreSample = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(45)), Math.toRadians(200))
                .build();

        TrajectorySequence collectSample1 = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-35, -25, Math.toRadians(180)), Math.toRadians(70))
                .build();

        TrajectorySequence Park = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-35, -10, Math.toRadians(180)), Math.toRadians(50))
                .build();

        TrajectorySequence Ascent = drive.trajectorySequenceBuilder(new Pose2d(-35, -10, Math.toRadians(180)))
                .back(10)
                .build();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        sleep(4);

        Robot.outtake.closeClaw(outtakeClaw);

        drive.setPoseEstimate(startPose);

        drive.followTrajectorySequence(scoreSample);

        slideMotor_right.setTargetPosition(-2980);
//            slideMotor2.setTargetPosition(0);

        slideMotor_right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            slideMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slideMotor_right.setPower(-1.0);
        slideMotor_left.setPower(-1.0);


        while (slideMotor_right.isBusy()){
            telemetry.addData("slides going up", "");
            telemetry.update();
        }

        slideMotor_right.setPower(0.0);
        slideMotor_left.setPower(0.0);

        Robot.outtake.scoreSample(outtakeArm);
        sleep(800);
        Robot.outtake.openClaw(outtakeClaw);
        sleep(500);

        Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeWrist);

        slideMotor_right.setTargetPosition(0);
//            slideMotor2.setTargetPosition(0);

        slideMotor_right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            slideMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slideMotor_right.setPower(1.0);
        slideMotor_left.setPower(1.0);


        while (slideMotor_right.isBusy()){
            telemetry.addData("slides going down", "");
            telemetry.update();
        }

        slideMotor_right.setPower(0.0);
        slideMotor_left.setPower(0.0);

        drive.followTrajectorySequence(collectSample1);

        linkage1.setPosition(0.17);
        linkage2.setPosition(0.972);
        Robot.intake.dropBucket(bucket);
        Robot.intake.runIntake(intakeRollers);
        sleep(500);

        linkage1.setPosition(0.197);
        linkage2.setPosition(0.92);
        sleep(1500);

        Robot.intake.transfer(linkage1, linkage2, bucket);
        sleep(1000);
        intakeRollers.setPower(0.0);
        Robot.outtake.closeClaw(outtakeClaw);

        drive.followTrajectorySequence(scoreSample);

        slideMotor_right.setTargetPosition(-2980);
//            slideMotor2.setTargetPosition(0);

        slideMotor_right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            slideMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slideMotor_right.setPower(-1.0);
        slideMotor_left.setPower(-1.0);


        while (slideMotor_right.isBusy()){
            telemetry.addData("slides going up", "");
            telemetry.update();
        }

        slideMotor_right.setPower(0.0);
        slideMotor_left.setPower(0.0);

        Robot.outtake.scoreSample(outtakeArm);
        sleep(800);
        Robot.outtake.openClaw(outtakeClaw);
        sleep(500);

        Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeWrist);

        slideMotor_right.setTargetPosition(0);
//            slideMotor2.setTargetPosition(0);

        slideMotor_right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            slideMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slideMotor_right.setPower(1.0);
        slideMotor_left.setPower(1.0);


        while (slideMotor_right.isBusy()){
            telemetry.addData("slides going down", "");
            telemetry.update();
        }

        slideMotor_right.setPower(0.0);
        slideMotor_left.setPower(0.0);

        drive.followTrajectorySequence(Park);

        Robot.outtake.scoreSample(outtakeArm);
        drive.followTrajectorySequence(Ascent);

        sleep(4000);

//        Robot.intake.fullExtend(linkage1, linkage2);
//        sleep(1000);
//        Robot.intake.dropBucket(bucket);
//        sleep(500);
//        Robot.intake.reverseIntake(intakeRollers);
//        sleep(1500);
//        Robot.intake.transfer(linkage1, linkage2, bucket);
//        sleep(1000);
//        drive.followTrajectorySequence(park);

        stop();


    }

}

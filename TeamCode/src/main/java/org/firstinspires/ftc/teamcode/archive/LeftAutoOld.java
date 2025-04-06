//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
//import com.fasterxml.jackson.annotation.JsonTypeInfo;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
//
////@Disabled
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous
//public class LeftAuto extends LinearOpMode {
//
//    @Override
//    public void runOpMode() {
//
//        Servo linkage1;
//        Servo linkage2;
//
//        Servo outtakeClaw;
//        Servo outtakeWrist;
//        Servo outtakeArm;
//
//        DcMotorEx slideMotor_left;
//        DcMotorEx slideMotor_right;
//        Servo intakeDiffyLeft;
//        Servo intakeDiffyRight;
//        Servo intakeArmLeft;
//        Servo intakeArmRight;
//        Servo intakeClaw;
//
//        linkage1 = hardwareMap.get(Servo.class, "linkage1");
//        linkage2 = hardwareMap.get(Servo.class, "linkage2");
//
//        outtakeClaw = hardwareMap.get(Servo.class,"outtakeClaw");
//        outtakeWrist = hardwareMap.get(Servo.class,"outtakeWrist");
//        outtakeArm = hardwareMap.get(Servo.class,"outtakeArm");
//
//        slideMotor_left = hardwareMap.get(DcMotorEx.class,"slideMotor_left");
//        slideMotor_right = hardwareMap.get(DcMotorEx.class,"slideMotor_right");
//        slideMotor_right.setDirection(DcMotorEx.Direction.REVERSE);
//        intakeArmLeft = hardwareMap.get(Servo.class, "intakeArmLeft");
//        intakeArmRight = hardwareMap.get(Servo.class, "intakeArmRight");
//        intakeDiffyLeft = hardwareMap.get(Servo.class, "intakeDiffyLeft");
//        intakeDiffyRight = hardwareMap.get(Servo.class, "intakeDiffyRight");
//        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
//
//        Robot.intake.transfer(linkage1, linkage2, intakeArmLeft, intakeArmRight, intakeDiffyLeft, intakeDiffyRight, intakeClaw);
//        Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeWrist);
//        Robot.outtake.closeClaw(outtakeClaw);
//        intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
//
//        ElapsedTime outtakeTimer = new ElapsedTime();
//
//        // Create Roadrunner Trajectories
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        Pose2d startPose = new Pose2d(-38, -61, Math.toRadians(0));
//
//        TrajectorySequence scoreSample = drive.trajectorySequenceBuilder(startPose)
//                .splineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(45)), Math.toRadians(200))
//                .build();
//
//        TrajectorySequence collectSample1 = drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(-57, -50, Math.toRadians(71)))
//                .build();
//
//        TrajectorySequence Park = drive.trajectorySequenceBuilder(startPose)
//                .splineToLinearHeading(new Pose2d(-35, -10, Math.toRadians(180)), Math.toRadians(50))
//                .build();
//
//        TrajectorySequence Ascent = drive.trajectorySequenceBuilder(new Pose2d(-35, -10, Math.toRadians(180)))
//                .back(10)
//                .build();
//
//        Robot.intake.transfer(linkage1, linkage2, intakeArmLeft, intakeArmRight, intakeDiffyLeft, intakeDiffyRight, intakeClaw);
//        Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeWrist);
//        Robot.outtake.closeClaw(outtakeClaw);
//        intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
//
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//        sleep(4);
//
//        drive.setPoseEstimate(startPose);
//
//        drive.followTrajectorySequence(scoreSample);
//        drive.followTrajectorySequence(collectSample1);
//
//        linkage1.setPosition(0.14);
//        linkage2.setPosition(0.86);
//
//        sleep(600);
//
//        intakeArmLeft.setPosition(Robot.INTAKE_ARM_LEFT_EXTEND);
//        intakeArmRight.setPosition(Robot.INTAKE_ARM_RIGHT_EXTEND);
//        intakeDiffyLeft.setPosition(Robot.INTAKE_LEFT_DIFFY_PICK_UP);
//        intakeDiffyRight.setPosition(Robot.INTAKE_RIGHT_DIFFY_PICK_UP);
//
//        sleep(1000);
//
//        intakeClaw.setPosition(Robot.INTAKE_CLAW_CLOSE);
//
//        sleep(4000);
//
//        stop();
//
//
//    }
//
//}

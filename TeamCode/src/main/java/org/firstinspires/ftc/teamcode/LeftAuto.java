package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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

        linkage1 = hardwareMap.get(Servo.class, "linkage1");
        linkage2 = hardwareMap.get(Servo.class, "linkage2");
        bucket = hardwareMap.get(Servo.class, "bucket");
        intakeRollers = hardwareMap.get(CRServo.class, "intakeRollers");

        Robot.intake.transfer(linkage1, linkage2, bucket);

        // Create Roadrunner Trajectories

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-38, -61, Math.toRadians(90));

        TrajectorySequence scoreSample = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-38, -40, Math.toRadians(220)))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-30, -10, Math.toRadians(90)))
                .build();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //sleep(4);

        sleep(4);

        drive.setPoseEstimate(startPose);

        drive.followTrajectorySequence(scoreSample);
        Robot.intake.fullExtend(linkage1, linkage2);
        sleep(1000);
        Robot.intake.dropBucket(bucket);
        sleep(500);
        Robot.intake.reverseIntake(intakeRollers);
        sleep(1500);
        Robot.intake.transfer(linkage1, linkage2, bucket);
        sleep(1000);
        drive.followTrajectorySequence(park);


    }

}

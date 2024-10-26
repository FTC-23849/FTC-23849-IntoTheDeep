package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class RightAuto extends LinearOpMode {

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

        Pose2d startPose = new Pose2d(15, -61, Math.toRadians(90));

        TrajectorySequence park = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(35)
                .build();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //sleep(4);

        sleep(4);

        drive.setPoseEstimate(startPose);

        drive.followTrajectorySequence(park);


    }

}

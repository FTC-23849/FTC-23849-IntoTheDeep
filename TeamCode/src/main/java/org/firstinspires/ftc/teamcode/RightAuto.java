package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

        Servo outtakeClaw;
        Servo outtakeWrist;
        Servo outtakeArm;

        DcMotorEx slideMotor_left;
        DcMotorEx slideMotor_right;
        Servo intakeDiffyLeft;
        Servo intakeDiffyRight;
        Servo intakeArmLeft;
        Servo intakeArmRight;
        Servo intakeClaw;

        linkage1 = hardwareMap.get(Servo.class, "linkage1");
        linkage2 = hardwareMap.get(Servo.class, "linkage2");
        bucket = hardwareMap.get(Servo.class, "bucket");
        intakeRollers = hardwareMap.get(CRServo.class, "intakeRollers");

        outtakeClaw = hardwareMap.get(Servo.class,"outtakeClaw");
        outtakeWrist = hardwareMap.get(Servo.class,"outtakeWrist");
        outtakeArm = hardwareMap.get(Servo.class,"outtakeArm");

        slideMotor_left = hardwareMap.get(DcMotorEx.class,"slideMotor_left");
        slideMotor_right = hardwareMap.get(DcMotorEx.class,"slideMotor_right");
        intakeArmLeft = hardwareMap.get(Servo.class, "intakeArmLeft");
        intakeArmRight = hardwareMap.get(Servo.class, "intakeArmRight");
        intakeDiffyLeft = hardwareMap.get(Servo.class, "intakeDiffyLeft");
        intakeDiffyRight = hardwareMap.get(Servo.class, "intakeDiffyRight");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        slideMotor_right.setDirection(DcMotorEx.Direction.REVERSE);

        Robot.intake.transfer(linkage1, linkage2, intakeArmLeft, intakeArmRight, intakeDiffyLeft, intakeDiffyRight, intakeClaw);
        Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeWrist);

        // Create Roadrunner Trajectories

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(9, -61, Math.toRadians(0));

        TrajectorySequence park = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(40, -57), Math.toRadians(0))
                .build();

        TrajectorySequence scoreSpecimen = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(9, -35, Math.toRadians(270)))
                .build();

        Robot.outtake.closeClaw(outtakeClaw);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //sleep(4);

        sleep(4);

        drive.setPoseEstimate(startPose);

        Robot.outtake.scoreSpecimen(outtakeArm, outtakeWrist, outtakeClaw);

        slideMotor_right.setTargetPosition(-2100);
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

        drive.followTrajectorySequence(scoreSpecimen);

        slideMotor_right.setTargetPosition(-900);
//            slideMotor2.setTargetPosition(0);

        slideMotor_right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            slideMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slideMotor_right.setPower(0.75);
        slideMotor_left.setPower(0.75);


        while (slideMotor_right.isBusy()){
            telemetry.addData("slides going down", "");
            telemetry.update();
        }

        slideMotor_right.setPower(0.0);
        slideMotor_left.setPower(0.0);

        Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeWrist);

        sleep(500);


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

        drive.followTrajectorySequence(park);


        sleep(4000);




    }

}

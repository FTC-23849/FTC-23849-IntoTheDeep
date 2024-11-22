package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class RightAuto0_2 extends LinearOpMode {

    @Override
    public void runOpMode() {

        Servo linkage1;
        Servo linkage2;

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

        int slidePosition;

        linkage1 = hardwareMap.get(Servo.class, "linkage1");
        linkage2 = hardwareMap.get(Servo.class, "linkage2");

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
        Robot.outtake.closeClaw(outtakeClaw);
        intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);

        // Create Roadrunner Trajectories

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(9, -61, Math.toRadians(0));

        TrajectorySequence scoreSpecimen = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(9, -35, Math.toRadians(270)))
                .build();

        TrajectorySequence pushSamples = drive.trajectorySequenceBuilder(new Pose2d(9, -35, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(35, -40, Math.toRadians(90)), Math.toRadians(85))
                .splineToLinearHeading(new Pose2d(43, -10, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(43, -50, Math.toRadians(90)), Math.toRadians(60))
                .splineToConstantHeading(new Vector2d(53, -10), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(56, -50, Math.toRadians(90)), Math.toRadians(40))
//                .lineToLinearHeading(new Pose2d(35, -40, Math.toRadians(90)))
//                .splineToConstantHeading(new Vector2d(45, -10), Math.toRadians(0))
//                .lineToLinearHeading(new Pose2d(47, -50, Math.toRadians(90)))
//                .splineToConstantHeading(new Vector2d(53, -10), Math.toRadians(0))
//                .lineToLinearHeading(new Pose2d(56, -50, Math.toRadians(90)))
                .build();

        TrajectorySequence collectSpecimen1 = drive.trajectorySequenceBuilder(new Pose2d(56, -50, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(37, -50))
                .lineToConstantHeading(new Vector2d(37, -57))
                .build();

        TrajectorySequence scoreNextSpecimen = drive.trajectorySequenceBuilder(new Pose2d(37, -57, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(-1, -35, Math.toRadians(270)), Math.toRadians(90))
                .build();

        TrajectorySequence collectNextSpecimen_park = drive.trajectorySequenceBuilder(new Pose2d(9, -35, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(37, -57, Math.toRadians(90)))
                .build();

        Robot.outtake.closeClaw(outtakeClaw);

        slideMotor_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //First Specimen

        //sleep(4);

        sleep(4);

        linkage1.setPosition(0.05);
        linkage2.setPosition(0.95);

        drive.setPoseEstimate(startPose);

        Robot.outtake.scoreSpecimen(outtakeArm, outtakeWrist, outtakeClaw);

        //slideMotor_right.setTargetPosition(-50);

        slideMotor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideMotor_right.setTargetPosition(-1350);

        slideMotor_right.setPower(-1.0);
        slideMotor_left.setPower(-1.0);

        while (slideMotor_right.isBusy()){
            telemetry.addData("slides going up", "");
            telemetry.update();
        }

        telemetry.addData("reached position", "");

        slideMotor_right.setPower(0.2);
        slideMotor_left.setPower(0.2);

        linkage1.setPosition(0.0);
        linkage2.setPosition(1.0);

        drive.followTrajectorySequence(scoreSpecimen);
        slideMotor_right.setTargetPosition(-800);

        slideMotor_right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slideMotor_right.setPower(1.0);
        slideMotor_left.setPower(1.0);


        while (slideMotor_right.isBusy()){
            telemetry.addData("slides going down", "");
            telemetry.update();
        }

        slideMotor_right.setPower(0.0);
        slideMotor_left.setPower(0.0);

        Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeWrist);

        sleep(500);


        slideMotor_right.setTargetPosition(0);

        slideMotor_right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slideMotor_right.setPower(1.0);
        slideMotor_left.setPower(1.0);


        while (slideMotor_right.isBusy()){
            telemetry.addData("slides going down", "");
            telemetry.update();
        }

        slideMotor_right.setPower(0.0);
        slideMotor_left.setPower(0.0);

        //pushing samples

        drive.followTrajectorySequence(pushSamples);

        //scoring 2nd specimen

        Robot.outtake.specimenReceivePosition(outtakeClaw, outtakeWrist, outtakeArm);

        drive.followTrajectorySequence(collectSpecimen1);

        Robot.outtake.closeClaw(outtakeClaw);

        slideMotor_right.setTargetPosition(-1400);

        slideMotor_right.setPower(-1.0);
        slideMotor_left.setPower(-1.0);

        while (slideMotor_right.isBusy()){
            telemetry.addData("slides going up", "");
            telemetry.update();
        }

        telemetry.addData("reached position", "");

        slideMotor_right.setPower(0.2);
        slideMotor_left.setPower(0.2);

        drive.followTrajectorySequence(scoreNextSpecimen);

        slideMotor_right.setTargetPosition(-650);

        slideMotor_right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slideMotor_right.setPower(1.0);
        slideMotor_left.setPower(1.0);


        while (slideMotor_right.isBusy()){
            telemetry.addData("slides going down", "");
            telemetry.update();
        }

        slideMotor_right.setPower(0.0);
        slideMotor_left.setPower(0.0);

        Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeWrist);

        sleep(500);


        slideMotor_right.setTargetPosition(0);

        slideMotor_right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slideMotor_right.setPower(1.0);
        slideMotor_left.setPower(1.0);


        while (slideMotor_right.isBusy()){
            telemetry.addData("slides going down", "");
            telemetry.update();
        }

        slideMotor_right.setPower(0.0);
        slideMotor_left.setPower(0.0);

        drive.followTrajectorySequence(collectNextSpecimen_park);

        sleep(4000);




    }

}

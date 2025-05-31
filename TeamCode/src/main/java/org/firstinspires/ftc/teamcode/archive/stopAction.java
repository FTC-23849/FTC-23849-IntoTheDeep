package org.firstinspires.ftc.teamcode.archive;

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

import org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class stopAction extends LinearOpMode {

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

        Robot.intake.transfer(linkage1, linkage2, intakeArmLeft, intakeArmRight, intakeDiffyLeft, intakeDiffyRight, intakeClaw);
        Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeArm2, outtakeWrist);
        Robot.outtake.closeClaw(outtakeClaw);
        intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
        intakeDiffyLeft.setPosition(0.72);
        intakeDiffyRight.setPosition(0.30);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){


        }

        Robot.outtake.openClaw(outtakeClaw);
        sleep(1000);


    }

}
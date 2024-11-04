package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    //Constants for intake
    public static double LINKAGE1_EXTEND = 0.28;
    public static double LINKAGE2_EXTEND = 0.72;

    public static double LINKAGE1_TRANSFER = 0.015;
    public static double LINKAGE2_TRANSFER = 0.975;

    public static double BUCKET_INTAKE_ANGLE = 0.54;
    public static double BUCKET_TRANSFER_ANGLE = 0.0;

    public static double INTAKE_SPEED = 1.0;

    //Constants for outtake
    public static double OPEN_CLAW = 0.1;
    public static double CLOSE_CLAW = 0.8;
    public static double SPECIMEN_SCORE_ORIENTATION = 0.0;
    public static double SPECIMEN_RECEIVE_ORIENTATION = 0.0;
    public static double SAMPLE_RECEIVE_ORIENTATION = 0.41;


    public static double ARM_SAMPLE_RECEIVE = 0.99;
    public static double ARM_SAMPLE_SCORE = 0.0;
    public static double ARM_SPECIMEN_RECEIVE = 0.0;
    public static double ARM_SPECIMEN_SCORE = 0.0;


    public static double SLIDE_SPEED = 0.75;


    public static class intake {
        public static void fullExtend(Servo linkage1, Servo linkage2){
            linkage1.setPosition(LINKAGE1_EXTEND);
            linkage2.setPosition(LINKAGE2_EXTEND);
        }

        public static void dropBucket(Servo bucket){
            bucket.setPosition(BUCKET_INTAKE_ANGLE);
        }

        public static void bucketUp(Servo bucket){
            bucket.setPosition(BUCKET_TRANSFER_ANGLE);
        }

        public static void transfer(Servo linkage1, Servo linkage2, Servo bucket){
            linkage1.setPosition(LINKAGE1_TRANSFER);
            linkage2.setPosition(LINKAGE2_TRANSFER);
            bucket.setPosition(BUCKET_TRANSFER_ANGLE);
        }

        public static void runIntake(CRServo intakeRollers){
            intakeRollers.setPower(INTAKE_SPEED);
        }

        public static void reverseIntake(CRServo intakeRollers){
            intakeRollers.setPower(INTAKE_SPEED * -1);
        }
    }

    public static class outtake {

        public static void runSlides(DcMotorEx slideMotor_left, DcMotorEx slideMotor_right){
            slideMotor_left.setPower(SLIDE_SPEED);
            slideMotor_right.setPower(-SLIDE_SPEED);
        }

        public static void reverseSlides(DcMotorEx slideMotor_left, DcMotorEx slideMotor_right){
            slideMotor_left.setPower(-SLIDE_SPEED);
            slideMotor_right.setPower(SLIDE_SPEED);
        }

        public static void openClaw(Servo outtakeClaw){
            outtakeClaw.setPosition(OPEN_CLAW);
        }

        public static void closeClaw(Servo outtakeClaw){
            outtakeClaw.setPosition(CLOSE_CLAW);
        }

        public static void sampleReceivePosition(Servo outtakeClaw, Servo outtakeArm, Servo outtakeWrist){
            outtakeClaw.setPosition(OPEN_CLAW);
            outtakeArm.setPosition(ARM_SAMPLE_RECEIVE);
            outtakeWrist.setPosition(SAMPLE_RECEIVE_ORIENTATION);
        }

        public static void specimenReceivePosition (Servo outtakeClaw){

        }

        public static void scoreSample(Servo outtakeArm){

            outtakeArm.setPosition(ARM_SAMPLE_SCORE);


        }

        public static void scoreSpecimen(Servo outtakeClaw){

        }
    }



}


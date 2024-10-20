package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    //Constants for intake
    public static double LINKAGE1_EXTEND = 0.3;
    public static double LINKAGE2_EXTEND = 0.7;

    public static double LINKAGE1_TRANSFER = 0.015;
    public static double LINKAGE2_TRANSFER = 0.975;

    public static double BUCKET_INTAKE_ANGLE = 0.60;
    public static double BUCKET_TRANSFER_ANGLE = 0.0;

    public static double INTAKE_SPEED = 1.0;

    //Constants for outtake
    public static double OPEN_CLAW = 0.0;
    public static double CLOSE_CLAW = 0.0;
    public static double SPECIMEN_SCORE_ORIENTATION = 0.0;
    public static double SPECIMEN_RECEIVE_ORIENTATION = 0.0;
    public static double SAMPLE_RECEIVE_ORIENTATION = 0.0;


    public static double ARM_SAMPLE = 0.0;
    public static double ELBOW_SAMPLE = 0.0;

    public static double ARM_SPECIMEN = 0.0;
    public static double ELBOW_SPECIMEN = 0.0;


    public static class intake {
        public static void fullExtend(Servo linkage1, Servo linkage2){
            linkage1.setPosition(LINKAGE1_EXTEND);
            linkage2.setPosition(LINKAGE2_EXTEND);
        }

        public static void dropBucket(Servo bucket){
            bucket.setPosition(BUCKET_INTAKE_ANGLE);
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
        public static void openClaw(Servo outtakeClaw){
            outtakeClaw.setPosition(OPEN_CLAW);
        }

        public static void closeClaw(Servo outtakeClaw){
            outtakeClaw.setPosition(CLOSE_CLAW);
        }

        public static void sampleReceivePosition(Servo outtakeClaw, Servo outtakeArm, Servo outtakeElbow, Servo outtakeWrist){

        }

        public static void specimenReceivePosition (Servo outtakeClaw){

        }

        public static void scoreSample(Servo outtakeClaw){

        }

        public static void scoreSpecimen(Servo outtakeClaw){

        }
    }



}


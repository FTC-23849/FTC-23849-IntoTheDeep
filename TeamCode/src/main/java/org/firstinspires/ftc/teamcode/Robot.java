package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    //Constants for intake
    public static double INTAKE_CLAW_OPEN = 0.4;
    public static double INTAKE_CLAW_CLOSE = 0.01;
    public static double INTAKE_LEFT_DIFFY_PICK_UP_READY = 0.49;
    public static double INTAKE_RIGHT_DIFFY_PICK_UP_READY = 0.51;
    public static double INTAKE_LEFT_DIFFY_PICK_UP_VERTICAL = 0.215;
    public static double INTAKE_RIGHT_DIFFY_PICK_UP_VERTICAL = 0.365;
    public static double INTAKE_LEFT_DIFFY_PICK_UP_45 = 0.317;
    public static double INTAKE_RIGHT_DIFFY_PICK_UP_45 = 0.472;
    public static double INTAKE_LEFT_DIFFY_PICK_UP_135 = 0.522;
    public static double INTAKE_RIGHT_DIFFY_PICK_UP_135 = 0.687;
    public static double INTAKE_LEFT_DIFFY_PICK_UP_180 = .625;
    public static double INTAKE_RIGHT_DIFFY_PICK_UP_180 = .795;
    public static double INTAKE_LEFT_DIFFY_PICK_UP =  0.42;
    public static double INTAKE_RIGHT_DIFFY_PICK_UP = 0.58;
    public static double INTAKE_LEFT_DIFFY_PICK_UP_WALL_SAMPLE = 0.325;
    public static double INTAKE_RIGHT_DIFFY_PICK_UP_WALL_SAMPLE = 0.465;
    public static double INTAKE_LEFT_DIFFY_TRANSFER = 0.81;
    public static double INTAKE_RIGHT_DIFFY_TRANSFER = 0.21;
    public static double INTAKE_LEFT_DIFFY_DROP = 0.63;
    public static double INTAKE_RIGHT_DIFFY_DROP = 0.38;
    public static double INTAKE_LEFT_DIFFY_0_180 = INTAKE_LEFT_DIFFY_PICK_UP_VERTICAL-INTAKE_LEFT_DIFFY_PICK_UP_180;
    public static double INTAKE_RIGHT_DIFFY_0_180 = INTAKE_RIGHT_DIFFY_PICK_UP_VERTICAL-INTAKE_RIGHT_DIFFY_PICK_UP_180;
    public static double INTAKE_ARM_LEFT_EXTEND = 0.07;
    public static double INTAKE_ARM_RIGHT_EXTEND = 0.93;
    public static double INTAKE_ARM_LEFT_EXTEND_TELEOP = 0.07;
    public static double INTAKE_ARM_RIGHT_EXTEND_TELEOP = 0.93;
    public static double INTAKE_ARM_LEFT_TRANSFER = 0.63;
    public static double INTAKE_ARM_RIGHT_TRANSFER = 0.37;
    public static double INTAKE_ARM_LEFT_EXTEND_READY = 0.15;
    public static double INTAKE_ARM_RIGHT_EXTEND_READY= 0.85;
    public static double INTAKE_ARM_LEFT_EXTEND_CAMERA = 0.21;
    public static double INTAKE_ARM_RIGHT_EXTEND_CAMERA= 0.79;
    public static double LINKAGE1_EXTEND = 0.22;
    public static double LINKAGE2_EXTEND = 0.79  ;

    public static double LINKAGE1_TRANSFER = 0.03;
    public static double LINKAGE2_TRANSFER = 0.98;

    //public static double BUCKET_INTAKE_ANGLE = 0.54;
    //public static double BUCKET_TRANSFER_ANGLE = 0.0;

    public static double INTAKE_SPEED = 1.0;

    //Constants for outtake
    public static double OPEN_CLAW = 0.5;
    public static double CLOSE_CLAW = 0.0;
    public static double SPECIMEN_RECEIVE_ORIENTATION = 0.6;
    public static double SPECIMEN_SCORE_ORIENTATION = 0.925;

    public static double SAMPLE_RECEIVE_ORIENTATION = 0.6;
    public static double SAMPLE_SCORE_ORIENTATION = 0.75;
    public static double SAMPLE_SCORE_ORIENTATION_TELEOP = 0.65;

    public static double ARM_SAMPLE_RECEIVE = 0.85;
    public static double ARM_SAMPLE_SCORE = 0.18;

    public static double ARM_SPECIMEN_RECEIVE = 0.14;
    //public static double ARM_SPECIMEN_RECEIVE_AUTO = 0.15;
    public static double ARM_SPECIMEN_SCORE = 0.82;

    public static double SUPPORT_SUPPORT = .7;
    public static double SUPPORT_RETRACT = 1.0;


    public static double SLIDE_SPEED = 1.0;


    public static class intake {
        public static void retractIntake(Servo linkage1, Servo linkage2, Servo intakeArmLeft, Servo intakeArmRight){
            linkage1.setPosition(Robot.LINKAGE1_TRANSFER);
            linkage2.setPosition(Robot.LINKAGE2_TRANSFER);
            intakeArmLeft.setPosition(Robot.INTAKE_ARM_LEFT_TRANSFER);
            intakeArmRight.setPosition(Robot.INTAKE_ARM_RIGHT_TRANSFER);
        }
        public static void fullExtend(Servo linkage1, Servo linkage2){
            linkage1.setPosition(LINKAGE1_EXTEND);
            linkage2.setPosition(LINKAGE2_EXTEND);
        }

        public static void dropBucket(/*Servo bucket*/) {
            //bucket.setPosition(BUCKET_INTAKE_ANGLE);
        }

        public static void bucketUp(/*Servo bucket*/){
            //bucket.setPosition(BUCKET_TRANSFER_ANGLE);
        }

        public static void transfer(Servo linkage1, Servo linkage2, Servo intakeArmLeft, Servo intakeArmRight, Servo intakeDiffyLeft, Servo intakeDiffyRight, Servo intakeClaw){
            intakeDiffyLeft.setPosition(Robot.INTAKE_LEFT_DIFFY_TRANSFER);
            intakeDiffyRight.setPosition(Robot.INTAKE_RIGHT_DIFFY_TRANSFER);
            linkage1.setPosition(LINKAGE1_TRANSFER);
            linkage2.setPosition(LINKAGE2_TRANSFER);
            intakeArmLeft.setPosition(Robot.INTAKE_ARM_LEFT_TRANSFER);
            intakeArmRight.setPosition(Robot.INTAKE_ARM_RIGHT_TRANSFER);
            intakeClaw.setPosition(Robot.INTAKE_CLAW_CLOSE);


            //bucket.setPosition(BUCKET_TRANSFER_ANGLE);
        }
        public static void transferNoRetract(Servo intakeArmLeft, Servo intakeArmRight, Servo intakeDiffyLeft, Servo intakeDiffyRight, Servo intakeClaw){
            intakeDiffyLeft.setPosition(Robot.INTAKE_LEFT_DIFFY_TRANSFER);
            intakeDiffyRight.setPosition(Robot.INTAKE_RIGHT_DIFFY_TRANSFER);
            intakeArmLeft.setPosition(Robot.INTAKE_ARM_LEFT_TRANSFER);
            intakeArmRight.setPosition(Robot.INTAKE_ARM_RIGHT_TRANSFER);
            intakeClaw.setPosition(Robot.INTAKE_CLAW_CLOSE);


        }

        public static void runIntake(/*CRServo intakeRollers*/){
            //intakeRollers.setPower(INTAKE_SPEED);
        }

        public static void reverseIntake(/*CRServo intakeRollers*/){
            //intakeRollers.setPower(INTAKE_SPEED * -1);
        }
    }

    public static class outtake {

        public static void runSlides(DcMotorEx slideMotor_left, DcMotorEx slideMotor_right, double ticks){
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

        public static void sampleReceivePosition(Servo outtakeClaw, Servo outtakeArm, Servo outtakeArm2, Servo outtakeWrist){
            outtakeClaw.setPosition(OPEN_CLAW);
            outtakeArm.setPosition(ARM_SAMPLE_RECEIVE);
            outtakeArm2.setPosition(ARM_SAMPLE_RECEIVE);
            outtakeWrist.setPosition(SAMPLE_RECEIVE_ORIENTATION);
        }

        public static void specimenReceivePositionPlain (Servo outtakeWrist, Servo outtakeArm, Servo outtakeArm2){
            outtakeArm.setPosition(ARM_SPECIMEN_RECEIVE);
            outtakeArm2.setPosition(ARM_SPECIMEN_RECEIVE);
            outtakeWrist.setPosition(SPECIMEN_RECEIVE_ORIENTATION);
        }

        public static void specimenReceivePosition (Servo outtakeClaw, Servo outtakeWrist, Servo outtakeArm, Servo outtakeArm2){
            outtakeArm.setPosition(ARM_SPECIMEN_RECEIVE);
            outtakeArm2.setPosition(ARM_SPECIMEN_RECEIVE);
            outtakeClaw.setPosition(OPEN_CLAW);
            outtakeWrist.setPosition(SPECIMEN_RECEIVE_ORIENTATION);
        }

//        public static void specimenReceivePositionAuto (Servo outtakeClaw, Servo outtakeWrist, Servo outtakeArm, Servo outtakeArm2){
//            outtakeArm.setPosition(ARM_SPECIMEN_RECEIVE_AUTO);
//            outtakeArm2.setPosition(ARM_SPECIMEN_RECEIVE_AUTO);
//            outtakeClaw.setPosition(OPEN_CLAW);
//            outtakeWrist.setPosition(SPECIMEN_RECEIVE_ORIENTATION);
//        }

        public static void scoreSample(Servo outtakeArm, Servo outtakeArm2, Servo outtakeWrist){
            outtakeArm.setPosition(ARM_SAMPLE_SCORE);
            outtakeArm2.setPosition(ARM_SAMPLE_SCORE);
            outtakeWrist.setPosition(SAMPLE_SCORE_ORIENTATION);
        }
        public static void scoreSampleTeleop(Servo outtakeArm, Servo outtakeArm2, Servo outtakeWrist){
            outtakeArm.setPosition(ARM_SAMPLE_SCORE);
            outtakeArm2.setPosition(ARM_SAMPLE_SCORE);
            outtakeWrist.setPosition(SAMPLE_SCORE_ORIENTATION_TELEOP);
        }

        public static void scoreSpecimen(Servo outtakeArm, Servo outtakeArm2, Servo outtakeWrist, Servo outtakeClaw){
            outtakeArm.setPosition(ARM_SPECIMEN_SCORE);
            outtakeArm2.setPosition(ARM_SPECIMEN_SCORE);
            outtakeWrist.setPosition(SPECIMEN_SCORE_ORIENTATION);
            outtakeClaw.setPosition(CLOSE_CLAW);
        }

        public static void drivingScoreSpecimen(Servo outtakeArm, Servo outtakeWrist, Servo outtakeClaw){
            outtakeArm.setPosition(0.65);
            outtakeWrist.setPosition(SPECIMEN_RECEIVE_ORIENTATION);
            outtakeClaw.setPosition(CLOSE_CLAW);
        }

        public static void retractSupport(Servo outtakeSupport){
            outtakeSupport.setPosition(SUPPORT_RETRACT);
        }

        public static void extendSupport(Servo outtakeSupport){
            outtakeSupport.setPosition(SUPPORT_SUPPORT);
        }

    }



}


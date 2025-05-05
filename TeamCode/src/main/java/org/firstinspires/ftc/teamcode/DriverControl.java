/*
Copyright 2023 FIRST Tech Challenge Team FTC

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;



/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp(name = "DriverControl", group = "stuff")

public class DriverControl extends OpMode {
    /* Declare OpMode members. */
    DcMotorEx leftFrontMotor;
    DcMotorEx rightFrontMotor;
    DcMotorEx leftBackMotor;
    DcMotorEx rightBackMotor;
    DcMotorEx slideMotor_left;
    DcMotorEx slideMotor_right;
    DcMotorEx slideMotor_up;
    DcMotorEx slideMotor_down;
    Servo linkage1;
    Servo linkage2;
    Servo bucket;
    Servo outtakeClaw;
    Servo outtakeWrist;
    Servo outtakeArm;
    Servo outtakeArm2;
    //CRServo intakeRollers;
    Servo intakeDiffyLeft;
    Servo intakeDiffyRight;
    Servo intakeArmLeft;
    Servo intakeArmRight;
    Servo intakeClaw;
    Servo outtakeSupport;



    ElapsedTime mStateTime = new ElapsedTime();
    ElapsedTime outtakeTimer = new ElapsedTime();
    ElapsedTime clawTimer = new ElapsedTime();
    ElapsedTime transferTimer = new ElapsedTime();
    ElapsedTime hanging = new ElapsedTime();
    ElapsedTime supportUp = new ElapsedTime();
    ElapsedTime supportDown = new ElapsedTime();
    ElapsedTime intakeRetract = new ElapsedTime();

    boolean outtakeArmUp = false;
    boolean intakeExtended = false;
    double linkage1Position = 0.015;
    double linkage2Position = 0.975;
    double lastServoExtend = 0;
    int slideMaxPosition = -3200;
    boolean goingDown = false;
    boolean goingToSpecimen = false;
    boolean goingToSpecimenPickup = false;
    boolean goingToSpecimenScore = false;
    boolean raiseSupport = false;
    boolean armDown = false;
    int intakePosition = 0;
    boolean intakeRetracted = false;
    boolean xButton = false;

    @Override
    public void init() {

        leftFrontMotor  = hardwareMap.get(DcMotorEx.class,"LF");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RF");
        leftBackMotor = hardwareMap.get(DcMotorEx.class,"LB");
        rightBackMotor = hardwareMap.get(DcMotorEx.class,"RB");
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);

        intakeArmLeft = hardwareMap.get(Servo.class, "intakeArmLeft");
        intakeArmRight = hardwareMap.get(Servo.class, "intakeArmRight");
        intakeDiffyLeft = hardwareMap.get(Servo.class, "intakeDiffyLeft");
        intakeDiffyRight = hardwareMap.get(Servo.class, "intakeDiffyRight");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        //intakeRollers  = hardwareMap.get(CRServo.class,"intakeRollers");
        //bucket = hardwareMap.get(Servo.class, "bucket");
        linkage1 = hardwareMap.get(Servo.class,"linkage1");
        linkage2 = hardwareMap.get(Servo.class,"linkage2");

        outtakeClaw = hardwareMap.get(Servo.class,"outtakeClaw");
        outtakeWrist = hardwareMap.get(Servo.class,"outtakeWrist");
        outtakeArm = hardwareMap.get(Servo.class,"outtakeArm");
        outtakeArm2 = hardwareMap.get(Servo.class,"outtakeArm2");
        outtakeSupport = hardwareMap.get(Servo.class,"outtakeSupport");

        slideMotor_left = hardwareMap.get(DcMotorEx.class,"slideMotor_left");
        slideMotor_right = hardwareMap.get(DcMotorEx.class,"slideMotor_right");
        slideMotor_right.setDirection(DcMotorEx.Direction.REVERSE);
        slideMotor_up = hardwareMap.get(DcMotorEx.class, "slideMotor_up");
        slideMotor_down = hardwareMap.get(DcMotorEx.class, "slideMotor_down");
        slideMotor_down.setDirection(DcMotorEx.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(BRAKE);
        leftBackMotor.setZeroPowerBehavior(BRAKE);
        rightFrontMotor.setZeroPowerBehavior(BRAKE);
        rightBackMotor.setZeroPowerBehavior(BRAKE);

        //Robot.intake.transfer(linkage1, linkage2, bucket);
        Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeArm2, outtakeWrist);
        Robot.outtake.retractSupport(outtakeSupport);

        slideMotor_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor_up.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor_down.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Robot.intake.transfer(linkage1, linkage2, intakeArmLeft, intakeArmRight, intakeDiffyLeft, intakeDiffyRight, intakeClaw);

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    int rightBumperTimes = 0;
    boolean rightBumperPressed = false;
    int leftBumperTimes = 0;
    boolean leftBumperPressed = false;
    boolean intakeClawOpened = true;
    boolean outtakeClawDropped = true;
    boolean intakeSmallExtend = false;
    boolean slowRelease = false;
    @Override
    public void loop() {
        double xDistance = 0;
        double yDistance = 0;
        double speed = 0;
        double direction = 0;
        int preciseSpeedDivider = 3;
        boolean preciseDriving;



//        telemetry.addData("voltage-leftFrontMotor", leftFrontMotor.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("voltage-leftBackMotor", leftBackMotor.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("voltage-rightFrontMotor", rightFrontMotor.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("voltage-rightBackMotor", rightBackMotor.getCurrent(CurrentUnit.AMPS));
//        telemetry.update();

//        telemetry.addData("LF", leftFrontMotor.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("LB", leftBackMotor.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("RF", rightFrontMotor.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("RB", rightBackMotor.getCurrent(CurrentUnit.AMPS));


//        if (gamepad1.right_bumper) {
//            Robot.intake.runIntake(intakeRollers);
//        } else if (gamepad1.left_bumper) {
//            Robot.intake.reverseIntake(intakeRollers);
//        } else {
//            intakeRollers.setPower(0.0);
//        }
//
//        if (gamepad1.right_trigger > 0.2) {
//            Robot.intake.fullExtend(linkage1, linkage2);
//        } else if (gamepad1.left_trigger > 0.2) {
//            Robot.intake.transfer(linkage1, linkage2, bucket);
//        }
//
//        if (gamepad1.a) {
//            Robot.intake.dropBucket(bucket);
//        }
        if (gamepad1.left_bumper) {
            if(leftBumperPressed == false){
                leftBumperTimes = leftBumperTimes + 1;
            }

            leftBumperPressed = true;
            if(leftBumperTimes == 1){

                intakeDiffyLeft.setPosition(Robot.INTAKE_LEFT_DIFFY_PICK_UP_VERTICAL);
                intakeDiffyRight.setPosition(Robot.INTAKE_RIGHT_DIFFY_PICK_UP_VERTICAL);

            }
            if(leftBumperTimes == 2){
                leftBumperTimes = 0;
                intakeDiffyLeft.setPosition(Robot.INTAKE_LEFT_DIFFY_PICK_UP);
                intakeDiffyRight.setPosition(Robot.INTAKE_RIGHT_DIFFY_PICK_UP);
            }
        }
        else{
            leftBumperPressed = false;
        }
        if (gamepad1.right_bumper) {
            if (intakeRetracted == false) {

                if (rightBumperPressed == false) {
                    rightBumperTimes = rightBumperTimes + 1;
                    //}
                    rightBumperPressed = true;
                    if (rightBumperTimes == 1) {
                /*Robot.intake.dropBucket(bucket);
                intakeRollers.setPower(0);*/
                        intakeArmLeft.setPosition(Robot.INTAKE_ARM_LEFT_EXTEND_TELEOP);
                        intakeArmRight.setPosition(Robot.INTAKE_ARM_RIGHT_EXTEND_TELEOP);
                    }
                    rightBumperPressed = true;


                }

                if (rightBumperTimes == 2) {
                    intakeClaw.setPosition(Robot.INTAKE_CLAW_CLOSE);
                    outtakeClaw.setPosition(Robot.OPEN_CLAW);
                    //Robot.intake.runIntake(intakeRollers);
                }
                if (rightBumperTimes == 3) {
                    intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
                    rightBumperTimes = 1;
                }
            }
            else{
                outtakeClaw.setPosition(Robot.OPEN_CLAW);

        }



        }
        else {
            rightBumperPressed = false;
        }






//        telemetry.addData("rightBumper", rightBumperTimes);
//        telemetry.addData("left bumper", leftBumperTimes);
//
//        telemetry.addData("lastServoExtend", lastServoExtend);
//        telemetry.addData("time", mStateTime.milliseconds());
//        telemetry.update();
        if (gamepad1.right_trigger > 0.2|| gamepad1.a){
            intakeArmLeft.setPosition(Robot.INTAKE_ARM_LEFT_EXTEND_READY);
            intakeArmRight.setPosition(Robot.INTAKE_ARM_RIGHT_EXTEND_READY);
            intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
            intakeDiffyLeft.setPosition(Robot.INTAKE_LEFT_DIFFY_PICK_UP);
            intakeDiffyRight.setPosition(Robot.INTAKE_RIGHT_DIFFY_PICK_UP);
            rightBumperTimes = 0;
            intakeRetracted = false;

        }
        if (gamepad1.right_trigger > 0.2) {
            /*Robot.intake.fullExtend(linkage1, linkage2);
            intakeExtended = true;*/
            intakeRetracted = false;





            if (mStateTime.milliseconds() - lastServoExtend > 10){
                //intakeArmLeft.setPosition(Robot.INTAKE_ARM_LEFT_EXTEND_READY);
                //intakeArmRight.setPosition(Robot.INTAKE_ARM_RIGHT_EXTEND_READY);
                linkage1.setPosition(linkage1Position+0.008);
                linkage2.setPosition(linkage2Position-0.008);
                linkage1Position = linkage1Position+0.008;
                linkage2Position = linkage2Position-0.008;
                lastServoExtend = mStateTime.milliseconds() ;
            }
            if(linkage1Position> Robot.LINKAGE1_EXTEND){
                linkage1.setPosition (/*0.17*/Robot.LINKAGE1_EXTEND);
                linkage2.setPosition (/*0.83*/Robot.LINKAGE2_EXTEND);
                linkage1Position = Robot.LINKAGE1_EXTEND;
                linkage2Position = Robot.LINKAGE2_EXTEND;
            }

        } else if (gamepad1.left_trigger > 0.2|| gamepad1.x) {
            Robot.intake.transferNoRetract(intakeArmLeft, intakeArmRight, intakeDiffyLeft, intakeDiffyRight, intakeClaw);
            //Robot.intake.retractIntake(inkage1, linkage2, intakeArmLeft, intakeArmRight);
            intakeRetracted = true;
            rightBumperTimes = 0;
            leftBumperTimes = 0;
            //intakeRollers.setPower(0);
            lastServoExtend = mStateTime.milliseconds() ;
            intakeRetract.reset();
            //linkage1Position = Robot.LINKAGE1_TRANSFER;
            //linkage2Position = Robot.LINKAGE2_TRANSFER;
            if(gamepad1.x){
                xButton = true;
            }
            else{
                xButton = false;
            }
        }

        if(intakeRetract.milliseconds()>100 && intakeRetract.milliseconds()<150){
            linkage1Position = Robot.LINKAGE1_TRANSFER;
            linkage2Position = Robot.LINKAGE2_TRANSFER;
            linkage1.setPosition(Robot.LINKAGE1_TRANSFER);
            linkage2.setPosition(Robot.LINKAGE2_TRANSFER);

        }
        if(intakeRetract.milliseconds()> 1000 && intakeRetract.milliseconds()<1050 && xButton == true){
            outtakeClaw.setPosition(Robot.CLOSE_CLAW);

        }
        if(intakeRetract.milliseconds()> 1100 && intakeRetract.milliseconds()<1150 && xButton == true){
            intakeClaw.setPosition((Robot.INTAKE_CLAW_OPEN));
        }
        if(gamepad1.dpad_down){
            intakeDiffyLeft.setPosition(Robot.INTAKE_LEFT_DIFFY_TRANSFER);
            intakeDiffyRight.setPosition(Robot.INTAKE_RIGHT_DIFFY_TRANSFER);
            linkage1.setPosition(Robot.LINKAGE1_TRANSFER);
            linkage2.setPosition(Robot.LINKAGE2_TRANSFER);
        }
        if (gamepad1.b){
            //Robot.intake.bucketUp(bucket);
            rightBumperTimes = 2;
            intakeArmLeft.setPosition(Robot.INTAKE_ARM_LEFT_EXTEND_READY);
            intakeArmRight.setPosition(Robot.INTAKE_ARM_RIGHT_EXTEND_READY);
            intakeDiffyLeft.setPosition(Robot.INTAKE_LEFT_DIFFY_DROP);
            intakeDiffyRight.setPosition(Robot.INTAKE_RIGHT_DIFFY_DROP);
            linkage1.setPosition(Robot.LINKAGE1_EXTEND);
            linkage2.setPosition(Robot.LINKAGE2_EXTEND);
            intakeRetracted = false;

        }


        //Control the outtake slides with the joystick
        if (gamepad2.right_stick_y < 0 && slideMotor_right.getCurrentPosition() > slideMaxPosition) {

            goingDown = false;
            goingToSpecimen = false;
            goingToSpecimenPickup = false;
            goingToSpecimenScore = false;

            slideMotor_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor_up.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor_down.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor_left.setPower(gamepad2.right_stick_y);
            slideMotor_right.setPower(gamepad2.right_stick_y);
            slideMotor_up.setPower(gamepad2.right_stick_y);
            slideMotor_down.setPower(gamepad2.right_stick_y);

        } else if (gamepad2.right_stick_y > 0) {

            goingDown = false;
            goingToSpecimen = false;
            goingToSpecimenPickup = false;
            slideMotor_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor_up.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor_down.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor_left.setPower(gamepad2.right_stick_y);
            slideMotor_right.setPower(gamepad2.right_stick_y);
            slideMotor_up.setPower(gamepad2.right_stick_y);
            slideMotor_down.setPower(gamepad2.right_stick_y);

        } else {

            if(goingDown == false && goingToSpecimen == false && goingToSpecimenPickup == false && goingToSpecimenScore == false) {
                slideMotor_left.setPower(0.0);
                slideMotor_right.setPower(0.0);
                slideMotor_up.setPower(0.0);
                slideMotor_down.setPower(0.0);
            }

        }

//        telemetry.addData("GoingDown =", goingDown);
//        telemetry.update();





        if (gamepad2.right_bumper) {
            outtakeClaw.setPosition(Robot.CLOSE_CLAW);
            transferTimer.reset();
            intakeClawOpened = false;

            //intakeRollers.setPower(0);
        }
        if(intakeClawOpened == false && transferTimer.milliseconds() > 150){
            intakeClaw.setPosition(Robot.INTAKE_CLAW_OPEN);
            intakeClawOpened = true;
        }
        if (gamepad2.left_bumper) {
            outtakeClaw.setPosition(Robot.OPEN_CLAW);
        }


        if (gamepad2.dpad_right && gamepad2.x) {
            slideMotor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor_up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor_down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor_up.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor_down.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (gamepad2.dpad_left && gamepad2.b) {
            slideMaxPosition = slideMotor_right.getCurrentPosition();
        }

        if (gamepad2.x) {

            raiseSupport = false;
            Robot.outtake.specimenReceivePosition(outtakeClaw, outtakeWrist, outtakeArm, outtakeArm2);
        }


        //Code to set outtake arm to rest position
//        if(gamepad2.dpad_down) {
//
//
//            Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeWrist);
//
//            slideMotor_right.setTargetPosition(0);
//
//            slideMotor_right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//            slideMotor_right.setPower(1.0);
//            slideMotor_left.setPower(1.0);
//
//            outtakeTimer.reset();
//
//            goingDown = true;
//
//            while ((slideMotor_right.isBusy() && slideMotor_right.getVelocity() > 0) && outtakeTimer.time() < 2/*&& slideMotor2.isBusy()*/){
//
//                outtakeTimer.reset();
//            }
//
//            // set motor power to zero to turn off motors
//
//            slideMotor_right.setPower(0.0);
//            slideMotor_left.setPower(0.0);
//
//        }

        if (gamepad2.dpad_down) {

            raiseSupport = false;

            Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeArm2, outtakeWrist);

            slideMotor_right.setTargetPosition(0);
            slideMotor_left.setTargetPosition(0);
            slideMotor_up.setTargetPosition(0);

            slideMotor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor_up.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideMotor_right.setPower(1.0);
            slideMotor_left.setPower(1.0);
            slideMotor_up.setPower(1.0);

            goingDown = true;
        }

        if (goingDown && slideMotor_right.getCurrentPosition() == 0 || slideMotor_left.getCurrentPosition() == 0 || slideMotor_up.getCurrentPosition() == 0){
            goingDown = false;
        }


        if (gamepad2.dpad_left) {

            raiseSupport = false;

            outtakeArm.setPosition(0.25);
            outtakeArm2.setPosition(0.25);
        }

//        if (gamepad2.dpad_right) {
//
//            raiseSupport = false;
//
//            outtakeArm.setPosition(0.15);
//            outtakeArm2.setPosition(0.12);
//        }

        if (gamepad2.y) {

            slideMotor_right.setTargetPosition(-1500);
            slideMotor_left.setTargetPosition(-1500);
            slideMotor_up.setTargetPosition(-1500);

            slideMotor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor_up.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideMotor_right.setPower(-1.0);
            slideMotor_left.setPower(-1.0);
            slideMotor_up.setPower(-1.0);

            goingToSpecimen = true;
            Robot.outtake.scoreSpecimen(outtakeArm, outtakeArm2, outtakeWrist, outtakeClaw);

            supportUp.reset();
            raiseSupport = true;

        }

        telemetry.addData("slideMotor_right", slideMotor_right.getCurrentPosition());
        telemetry.addData("slideMotor_left", slideMotor_left.getCurrentPosition());
        telemetry.addData("slideMotor_up", slideMotor_up.getCurrentPosition());

        if (supportUp.milliseconds() > 800 && raiseSupport == true) {

            Robot.outtake.extendSupport(outtakeSupport);

        } else {
            Robot.outtake.retractSupport(outtakeSupport);
        }

        if (gamepad2.a) {

            slideMotor_right.setTargetPosition(-800);
            slideMotor_left.setTargetPosition(-800);
            slideMotor_up.setTargetPosition(-800);

            slideMotor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor_up.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideMotor_right.setPower(-1.0);
            slideMotor_left.setPower(-1.0);
            slideMotor_up.setPower(-1.0);

            goingToSpecimenScore = true;


        }

        if (goingToSpecimenScore && (slideMotor_right.getCurrentPosition() == -800 || slideMotor_left.getCurrentPosition() == -800 || slideMotor_up.getCurrentPosition() == -800)){
            goingToSpecimenScore = false;
        }

        if (gamepad2.b) {

            Robot.outtake.specimenReceivePosition(outtakeClaw, outtakeWrist, outtakeArm, outtakeArm2);

            slideMotor_right.setTargetPosition(0);
            slideMotor_left.setTargetPosition(0);
            slideMotor_up.setTargetPosition(0);

            slideMotor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor_up.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideMotor_right.setPower(1.0);
            slideMotor_left.setPower(1.0);
            slideMotor_up.setPower(1.0);

            goingToSpecimenPickup = true;
            raiseSupport = false;

            Robot.outtake.retractSupport(outtakeSupport);

            supportDown.reset();
            armDown = true;

        }

        if (supportDown.milliseconds() > 600 && armDown == true) {
            Robot.outtake.specimenReceivePosition(outtakeClaw, outtakeWrist, outtakeArm, outtakeArm2);
            armDown = false;
        }






        if (goingDown && (slideMotor_right.getCurrentPosition() == 0 || slideMotor_left.getCurrentPosition() == 0 || slideMotor_up.getCurrentPosition() == 0)){
            goingToSpecimenPickup = false;
        }






//        if (gamepad2.a && gamepad2.right_bumper) {
//
//            hanging = true;
//
//            slideMotor_right.setTargetPosition(slideMotor_right.getCurrentPosition()-200);
//
//            slideMotor_right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//            slideMotor_right.setPower(1.0);
//            slideMotor_left.setPower(1.0);
//
//            outtakeTimer.reset();
//
//            goingDown = true;
//
//            while ((slideMotor_right.isBusy() && slideMotor_right.getVelocity() > 0) && outtakeTimer.time() < 2/*&& slideMotor2.isBusy()*/){
//                telemetry.addData("Slides going down", "");
//                telemetry.addData("slideMotor_right", slideMotor_right.getCurrentPosition());
//                telemetry.addData("slideMotor_left", slideMotor_left.getCurrentPosition());
//                telemetry.update();
//                outtakeTimer.reset();
//            }
//
//
//            telemetry.addData("slides all the way down", "");
//            telemetry.addData("slideMotor_right end position", slideMotor_right.getCurrentPosition());
////            telemetry.addData("slideMotor2 end position", slideMotor2.getCurrentPosition());
//            telemetry.update();
//
//            // set motor power to zero to turn off motors
//
//
//            while (hanging) {
//                slideMotor_right.setPower(0.3);
//                slideMotor_left.setPower(0.3);
//            }
//
//
//        }


        if (gamepad2.dpad_right && gamepad2.left_bumper) {
            hanging.reset();


            while (hanging.milliseconds() < 4000) {
                slideMotor_right.setPower(0.6);
                slideMotor_left.setPower(0.6);
                slideMotor_up.setPower(0.6);
            }

            while (hanging.milliseconds() > 4000 && hanging.milliseconds() < 13000 ) {
                slideMotor_right.setPower(0.1);
                slideMotor_left.setPower(0.1);
                slideMotor_up.setPower(0.1);
            }
        }

        if (gamepad2.dpad_up) {
            raiseSupport = false;
//            linkage1.setPosition(0.05);
//            linkage2.setPosition(0.95);
            Robot.outtake.scoreSampleTeleop(outtakeArm, outtakeArm2, outtakeWrist);
//            outtakeTimer.reset();
//            intakeSmallExtend = true;
        }
//        if(gamepad2.a){
//            outtakeSupport.setPosition(Robot.SUPPORT_SUPPORT);
//        }
//        if(gamepad2.b){
//            outtakeSupport.setPosition(Robot.SUPPORT_RETRACT);
//        }

//        if(intakeSmallExtend == true && transferTimer.milliseconds() > 200) {
//            linkage1.setPosition(0);
//            linkage2.setPosition(1.0);
//            intakeSmallExtend = false;
//
//        }
        /*if(gamepad2.y){
            slideMotor_left.setPower(0.8);
            slideMotor_right.setPower(0.8);
            slowRelease = true;
        }
        if(gamepad2.b){

            slowRelease = false;
        }*/

        //drivetrain
        if (gamepad1.left_stick_x < 0){
            xDistance = -gamepad1.left_stick_x;
        }
        else {
            xDistance = gamepad1.left_stick_x;
        }

        if (gamepad1.left_stick_y < 0) {
            yDistance = -gamepad1.left_stick_y;
        }
        else {
            yDistance = gamepad1.left_stick_y;
        }

        if (gamepad1.right_trigger > 0) {
            speed = (xDistance + yDistance) / preciseSpeedDivider;
            preciseDriving = true;
        }
        else {
            speed = xDistance + yDistance;
            preciseDriving = false;
        }

        // 57.29577951 = 1 radian = 180/pi
        if ((Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y) * 57.29577951) < 0){
            direction = (Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y)) * 57.29577951;
            direction += 360;
        }
        else {
            direction = (Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y)) * 57.29577951;
        }

        double joystickDriftTolerance = 0.05;
        if(Math.abs(speed) > joystickDriftTolerance) {
            drive(direction, speed, gamepad1.right_stick_x * speed, -gamepad1.right_stick_x * speed);
        }
        else {
            double turningSpeed = gamepad1.right_stick_x / 1.2;
            turn(turningSpeed);
        }

        telemetry.update();





    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //myRobot.OuttakeArmRest(elbowServo, wristServo, outtakeDoorServo, outtakeWheelServo);
    }
    public void drive (double direction, double speed, double leftOffset, double rightOffset){
//        leftFrontMotor  = hardwareMap.get(DcMotorEx.class,"LF");
//        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RF");
//        leftBackMotor = hardwareMap.get(DcMotorEx.class,"LB");
//        rightBackMotor = hardwareMap.get(DcMotorEx.class,"RB");
//        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
//        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
//
//        leftFrontMotor.setZeroPowerBehavior(BRAKE);
//        leftBackMotor.setZeroPowerBehavior(BRAKE);
//        rightFrontMotor.setZeroPowerBehavior(BRAKE);
//        rightBackMotor.setZeroPowerBehavior(BRAKE);

        double difference = 0;
        double lf = 0;
        double rf = 0;
        double lb = 0;
        double rb = 0;

        if(direction >= 0 && direction < 45){
            difference = 10 - direction / 4.5;
            rf = difference / 10 * speed;
            lb = difference / 10 * speed;
            rb = speed;
            lf = speed;
        }
        else if(direction >= 45 && direction < 90){
            difference = (direction - 45) * (1 / -4.5);
            rf = difference / 10 * speed;
            lb = difference / 10 * speed;
            rb = speed;
            lf = speed;
        }
        else if(direction >= 90 && direction < 135){
            difference = 10 - (direction - 90) / 4.5;
            rf = -speed;
            lb = -speed;
            rb = difference / 10 * speed;
            lf = difference / 10 * speed;
        }
        else if(direction >= 135 && direction < 180){
            difference = (direction - 135) * (1 / -4.5);
            rf = -speed;
            lb = -speed;
            rb = difference / 10 * speed;
            lf = difference / 10 * speed;
        }
        else if(direction >= 180 && direction < 225){
            difference = -10 - (direction - 180) / -4.5;
            rf = difference / 10 * speed;
            lb = difference / 10 * speed;
            rb = -speed;
            lf = -speed;
        }
        else if (direction >= 225 && direction < 270){
            difference = (direction-225) * (1 / 4.5);
            rf = difference / 10 * speed;
            lb = difference / 10 * speed;
            rb = -speed;
            lf = -speed;
        }
        else if (direction >= 270 && direction < 315){
            difference = -10 - (direction - 270) / -4.5;
            rf = speed;
            lb = speed;
            rb = difference / 10 * speed;
            lf = difference / 10 * speed;
        }
        else {
            difference = (direction - 315) * (1 / 4.5);
            rf = speed;
            lb = speed;
            rb = difference / 10 * speed;
            lf = difference / 10 * speed;
        }

        rf = rf + rightOffset;
        rb = rb + rightOffset;
        lf = lf + leftOffset;
        lb = lb + leftOffset;




            leftFrontMotor.setPower(lf);
            leftBackMotor.setPower(lb);
            rightFrontMotor.setPower(rf);
            rightBackMotor.setPower(rb);


    }
    public void turn ( double speed){
        if (intakeExtended == true){
            speed = speed*0.5;
        }

        leftFrontMotor  = hardwareMap.get(DcMotorEx.class,"LF");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RF");
        leftBackMotor = hardwareMap.get(DcMotorEx.class,"LB");
        rightBackMotor = hardwareMap.get(DcMotorEx.class,"RB");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightFrontMotor.setPower(speed*-1);
        rightBackMotor.setPower(speed*-1);

    }
}
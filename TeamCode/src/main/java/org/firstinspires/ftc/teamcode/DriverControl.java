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

import static org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit.VOLTS;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


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
    Servo linkage1;
    Servo linkage2;
    Servo bucket;
    Servo outtakeClaw;
    Servo outtakeWrist;
    Servo outtakeArm;
    CRServo intakeRollers;



    ElapsedTime mStateTime = new ElapsedTime();
    ElapsedTime outtakeTimer = new ElapsedTime();
    ElapsedTime clawTimer = new ElapsedTime();

    boolean outtakeArmUp = false;
    boolean intakeExtended = false;
    double linkage1Position = 0.015;
    double linkage2Position = 0.975;
    double lastServoExtend = 0;
    int slideMaxPosition = -3230;
    boolean outtakeGoingDown = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFrontMotor  = hardwareMap.get(DcMotorEx.class,"LF");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RF");
        leftBackMotor = hardwareMap.get(DcMotorEx.class,"LB");
        rightBackMotor = hardwareMap.get(DcMotorEx.class,"RB");
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeRollers  = hardwareMap.get(CRServo.class,"intakeRollers");
        bucket = hardwareMap.get(Servo.class, "bucket");
        linkage1 = hardwareMap.get(Servo.class,"linkage1");
        linkage2 = hardwareMap.get(Servo.class,"linkage2");

        outtakeClaw = hardwareMap.get(Servo.class,"outtakeClaw");
        outtakeWrist = hardwareMap.get(Servo.class,"outtakeWrist");
        outtakeArm = hardwareMap.get(Servo.class,"outtakeArm");

        slideMotor_left = hardwareMap.get(DcMotorEx.class,"slideMotor_left");
        slideMotor_right = hardwareMap.get(DcMotorEx.class,"slideMotor_right");
        slideMotor_right.setDirection(DcMotorEx.Direction.REVERSE);


        leftFrontMotor.setZeroPowerBehavior(BRAKE);
        leftBackMotor.setZeroPowerBehavior(BRAKE);
        rightFrontMotor.setZeroPowerBehavior(BRAKE);
        rightBackMotor.setZeroPowerBehavior(BRAKE);

        Robot.intake.transfer(linkage1, linkage2, bucket);
        Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeWrist);

        slideMotor_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);




    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    int rightBumperTimes = 0;
    boolean rightBumperPressed = false;
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

        telemetry.addData("LF", leftFrontMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("LB", leftBackMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("RF", rightFrontMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("RB", rightBackMotor.getCurrent(CurrentUnit.AMPS));


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

        if (gamepad1.right_bumper) {
            if(rightBumperPressed == false){
                rightBumperTimes = rightBumperTimes + 1;
            }
             rightBumperPressed = true;





            if (rightBumperTimes == 1) {
                Robot.intake.dropBucket(bucket);
                intakeRollers.setPower(0);
            }

            if (rightBumperTimes == 2) {
                Robot.intake.runIntake(intakeRollers);
            }

        } else if (gamepad1.left_bumper) {
            Robot.intake.reverseIntake(intakeRollers);
            rightBumperTimes = 1;

        }else if (gamepad1.a){
            intakeRollers.setPower(0.0);
        }


        else{
        rightBumperPressed = false;
        }

        telemetry.addData("rightBumper", rightBumperTimes);

        telemetry.addData("lastServoExtend", lastServoExtend);
        telemetry.addData("time", mStateTime.milliseconds());
        telemetry.update();
        if (gamepad1.right_trigger > 0.2) {
            /*Robot.intake.fullExtend(linkage1, linkage2);
            intakeExtended = true;*/

            if (mStateTime.milliseconds() - lastServoExtend > 15){
                linkage1.setPosition(linkage1Position+0.05);
                linkage2.setPosition(linkage2Position-0.005);
                linkage1Position = linkage1Position+0.005;
                linkage2Position = linkage2Position-0.005;
                lastServoExtend = mStateTime.milliseconds() ;
            }
            if(linkage1Position> 0.22){
                linkage1.setPosition (0.22);
                linkage2.setPosition (0.78);
                linkage1Position = 0.22;
                linkage2Position = 0.78;
            }

        } else if (gamepad1.left_trigger > 0.2) {
            Robot.intake.transfer(linkage1, linkage2, bucket);
            intakeExtended = false;
            rightBumperTimes = 0;
            //intakeRollers.setPower(0);
            lastServoExtend = mStateTime.milliseconds() ;
            linkage1Position = 0.0;
            linkage2Position = 1;
        }
        if (gamepad1.b){
            Robot.intake.bucketUp(bucket);
            rightBumperTimes = 0;
        }

        if (gamepad2.right_stick_y < 0 && slideMotor_right.getCurrentPosition() > slideMaxPosition) {
            slideMotor_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor_left.setPower(gamepad2.right_stick_y);
            slideMotor_right.setPower(gamepad2.right_stick_y);

        } else if (gamepad2.right_stick_y > 0) {
                slideMotor_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slideMotor_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slideMotor_left.setPower(gamepad2.right_stick_y);
                slideMotor_right.setPower(gamepad2.right_stick_y);
        } else {
            slideMotor_left.setPower(0.0);
            slideMotor_right.setPower(0.0);
        }


        if (gamepad2.right_bumper) {
            outtakeClaw.setPosition(Robot.CLOSE_CLAW);
            intakeRollers.setPower(0);
        }
        if (gamepad2.left_bumper) {
            outtakeClaw.setPosition(Robot.OPEN_CLAW);
            clawTimer.reset();
            while (clawTimer.milliseconds() < 500){

            }
            outtakeClaw.setPosition(Robot.CLOSE_CLAW);
        }

        if (gamepad2.a){
            outtakeClaw.setPosition(Robot.OPEN_CLAW);
        }


        if (gamepad2.dpad_right && gamepad2.x) {
            slideMotor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (gamepad2.dpad_left && gamepad2.b) {
            slideMaxPosition = slideMotor_right.getCurrentPosition();
        }

        if (gamepad2.x) {
            Robot.outtake.specimenReceivePosition(outtakeClaw, outtakeWrist, outtakeArm);
        }

        //Code to set outtake arm to rest position
        if(gamepad2.dpad_down) {


            Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeWrist);

            slideMotor_right.setTargetPosition(0);
//            slideMotor2.setTargetPosition(0);

            slideMotor_right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            slideMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            slideMotor_right.setPower(1.0);
            slideMotor_left.setPower(1.0);

//            outtakeTimer.reset();
//            while (outtakeTimer.time() < 0.2){
//
//            }

            outtakeTimer.reset();
            //outtakeGoingDown = true;
//
            while ((slideMotor_right.isBusy() && slideMotor_right.getVelocity() > 0) && outtakeTimer.time() < 2/*&& slideMotor2.isBusy()*/){
                telemetry.addData("Slides going down", "");
                telemetry.addData("slideMotor_right", slideMotor_right.getCurrentPosition());
                telemetry.addData("slideMotor_left", slideMotor_left.getCurrentPosition());
                telemetry.update();
                outtakeTimer.reset();
            }


            telemetry.addData("slides all the way down", "");
            telemetry.addData("slideMotor_right end position", slideMotor_right.getCurrentPosition());
//            telemetry.addData("slideMotor2 end position", slideMotor2.getCurrentPosition());
            telemetry.update();

            // set motor power to zero to turn off motors

            slideMotor_right.setPower(0.0);
            slideMotor_left.setPower(0.0);

        }
        /*if(outtakeGoingDown && !(slideMotor_right.getCurrentPosition() >= 2 && outtakeTimer.time() < 2)){
            slideMotor_right.setPower(0.0);
            slideMotor_left.setPower(0.0);
            outtakeGoingDown = false;

        }*/

        if (gamepad2.dpad_up) {
            Robot.outtake.scoreSample(outtakeArm);
        }


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
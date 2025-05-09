package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous

public class distanceSensorTest extends OpMode {
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
    Servo outtakeClaw;
    Servo outtakeWrist;
    Servo outtakeArm;
    Servo outtakeArm2;
    Servo intakeDiffyLeft;
    Servo intakeDiffyRight;
    Servo intakeArmLeft;
    Servo intakeArmRight;
    Servo intakeClaw;
    Servo outtakeSupport;

    DistanceSensor distSensor;

    double currentDistance;
    double speed = 0.3;
    int targetDistance = 70;
    double distanceAfterLoop;

    @Override
    public void init() {

        distSensor = hardwareMap.get(DistanceSensor.class, "distSensor");

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

        Robot.outtake.sampleReceivePosition(outtakeClaw, outtakeArm, outtakeArm2, outtakeWrist);
        Robot.outtake.retractSupport(outtakeSupport);

        slideMotor_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor_up.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor_down.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Robot.intake.transfer(linkage1, linkage2, intakeArmLeft, intakeArmRight, intakeDiffyLeft, intakeDiffyRight, intakeClaw);

    }

    @Override
    public void loop() {

//        currentDistance = distSensor.getDistance(DistanceUnit.MM);
//        telemetry.addData("current distance", currentDistance);
//
//        telemetry.addData("speed", speed);
//        telemetry.addData("targetDistance", targetDistance);
//        telemetry.update();

        if (gamepad1.dpad_up) {
            if (speed != 1.0) {
                speed = speed + 0.1;
            }
        }

        if (gamepad1.dpad_down) {
            if (speed != 0.1) {
                speed = speed - 0.1;
            }
        }

        if (gamepad1.dpad_left) {
            if (targetDistance != 1) {
                targetDistance --;
            }
        }

        if (gamepad1.dpad_right) {
            targetDistance ++;
        }
//        telemetry.clearAll();
        distanceAfterLoop = distSensor.getDistance(DistanceUnit.MM);
//
//        telemetry.addData("distance after loop", distanceAfterLoop);
//        telemetry.update();

        leftFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        rightBackMotor.setPower(speed);

        while ((distanceAfterLoop >= (targetDistance - 100)) && (distanceAfterLoop <= (targetDistance + 1))) {
            leftFrontMotor.setPower(0.0);
            leftBackMotor.setPower(0.0);
            rightFrontMotor.setPower(0.0);
            rightBackMotor.setPower(0.0);
            distanceAfterLoop = distSensor.getDistance(DistanceUnit.MM);
//            telemetry.addData("distance after loop", distanceAfterLoop);
//            telemetry.update();

        }
        //telemetry.update();
    }

}
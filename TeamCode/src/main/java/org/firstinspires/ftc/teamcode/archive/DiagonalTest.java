package org.firstinspires.ftc.teamcode.archive;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rrfiles.MecanumDrive;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class DiagonalTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Create Roadrunner Trajectories

        Pose2d startPose = new Pose2d(47, -60, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        TrajectoryActionBuilder test = drive.actionBuilder(startPose)
//                .setTangent(159.6)
                .strafeTo(new Vector2d(0, -35), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70))
//                .setTangent(159.6)
                .strafeTo(new Vector2d(35, -60), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70))

//                .setTangent(159.62)
                .strafeTo(new Vector2d(-2, -35), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70))
//                .setTangent(159.62)
                .strafeTo(new Vector2d(35, -60), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70));
//
//                .setTangent(159.64)
//                .lineToY(-35, new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70))
//                .setTangent(159.64)
//                .lineToY(-60, new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70))
//
//                .setTangent(159.66)
//                .lineToY(-35, new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70))
//                .setTangent(159.66)
//                .lineToY(-60, new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70))
//
//                .setTangent(159.68)
//                .lineToY(-35, new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70))
//                .setTangent(159.68)
//                .lineToY(-60, new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70));

        waitForStart();

        if (isStopRequested()) return;

        sleep(4);

        Actions.runBlocking(test.build());

        sleep(1000);


    }

}
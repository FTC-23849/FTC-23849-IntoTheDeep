package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(49.39, 60, Math.toRadians(180), Math.toRadians(180), 18.05)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(/*RightAuto Start Position*/ new Pose2d(9, -61, Math.toRadians(0))
                                                                                   /*LeftAuto Start Position*/ /*new Pose2d(-38, -61, Math.toRadians(0))*/)
                        //RightAuto Paths
                        .lineToLinearHeading(new Pose2d(9, -35, Math.toRadians(270)))
                        .splineToLinearHeading(new Pose2d(35, -40, Math.toRadians(90)), Math.toRadians(85))
                        .splineToLinearHeading(new Pose2d(43, -10, Math.toRadians(90)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(43, -50, Math.toRadians(90)), Math.toRadians(60))
                        .splineToConstantHeading(new Vector2d(53, -10), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(56, -50, Math.toRadians(90)), Math.toRadians(40))
                        .splineToConstantHeading(new Vector2d(37, -55), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(-1, -35, Math.toRadians(270)), Math.toRadians(180))
                        .lineToLinearHeading(new Pose2d(37, -57, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(9, -35, Math.toRadians(270)))
                        .lineToLinearHeading(new Pose2d(37, -57, Math.toRadians(90)))

                        //LeftAuto Paths
                        /*.splineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(45)), Math.toRadians(200))
                        .lineToLinearHeading(new Pose2d(-57, -50, Math.toRadians(67)))
                        .lineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(45)))
                        .lineToLinearHeading(new Pose2d(-57, -50, Math.toRadians(93)))
                        .lineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(45)))
                        .lineToLinearHeading(new Pose2d(-57, -40, Math.toRadians(130)))
                        .lineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(45)))
                        .splineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(180)), Math.toRadians(0))
                        .back(6)*/
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
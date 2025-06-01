package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 14)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-38, -61, Math.toRadians(0)))
                        .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.toRadians(200))
                        .lineToLinearHeading(new Pose2d(-52, -37, Math.toRadians(65)))
                        .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), Math.toRadians(90))
                        .lineToLinearHeading(new Pose2d(-57.5, -37, Math.toRadians(98)))
                        .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), Math.toRadians(90))
                        .lineToLinearHeading(new Pose2d(-57, -33, Math.toRadians(140)))
                        .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(-17, -5, Math.toRadians(180)), Math.toRadians(-10))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
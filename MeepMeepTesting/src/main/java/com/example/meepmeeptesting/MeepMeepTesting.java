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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-38, -61, Math.toRadians(0)))
                        .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.toRadians(200))
                        .splineToLinearHeading(new Pose2d(-35, -25, Math.toRadians(180)), Math.toRadians(70))
                        .splineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(45)), Math.toRadians(200))
                        .splineToLinearHeading(new Pose2d(-35, -10, Math.toRadians(180)), Math.toRadians(50))
                        .back(10)
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
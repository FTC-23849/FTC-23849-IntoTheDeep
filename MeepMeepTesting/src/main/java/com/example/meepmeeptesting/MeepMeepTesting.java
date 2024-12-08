package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(9, -61, Math.toRadians(270)))
                .lineToY(-38)
                .splineToLinearHeading(new Pose2d(35, -40, Math.toRadians(90)), Math.toRadians(80))
                .splineToConstantHeading(new Vector2d(43, -10), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(55, -55), Math.toRadians(30))
                .setTangent(90)
                .splineToConstantHeading(new Vector2d(55, -10), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60, -55), Math.toRadians(10))
                .splineToConstantHeading(new Vector2d(47, -48), Math.toRadians(180))
                .setTangent(1.6)
                .lineToY(-57)
                        .setTangent(15)
                .splineToLinearHeading(new Pose2d(7, -38, Math.toRadians(270)), Math.toRadians(90))
                        .setTangent(0)
                .splineToLinearHeading(new Pose2d(47, -52, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(1.6)
                .lineToY(-57)
                .splineToLinearHeading(new Pose2d(50, -55, Math.toRadians(90)), Math.toRadians(90))

//                .setTangent(90)
//                .splineToConstantHeading(new Vector2d(60, -10), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(70, -55), Math.toRadians(30))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
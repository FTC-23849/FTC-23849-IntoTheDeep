package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-38, -61, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(45)), Math.toRadians(200))
                .splineToLinearHeading(new Pose2d(-57, -50, Math.toRadians(71)), Math.toRadians(250))
                .splineToLinearHeading(new Pose2d(-50, -50, Math.toRadians(45)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-57, -50, Math.toRadians(94)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(45)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-50, -43, Math.toRadians(140)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(45)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-25, -10, Math.toRadians(180)), Math.toRadians(-10))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
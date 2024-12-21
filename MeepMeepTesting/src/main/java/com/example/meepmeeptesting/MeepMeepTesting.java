package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 14)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(9, -61, Math.toRadians(270)))
                    .lineToY(-33)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(31, -40, Math.toRadians(90)), Math.toRadians(10), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70))
                .splineToConstantHeading(new Vector2d(50, -10), Math.toRadians(10), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70))
                .setTangent(Math.toRadians(270))
                .lineToY(-53, new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100, 100))
                .setTangent(Math.toRadians(270))
                .lineToY(-10, new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100, 100))
                .setTangent(Math.toRadians(0))
                .lineToX(59, new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100, 100))
                .setTangent(Math.toRadians(270))
                .lineToY(-53, new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100, 100))
                .lineToY(-59, new TranslationalVelConstraint(15), new ProfileAccelConstraint(-15, 15))
                .setTangent(15)
                .splineToLinearHeading(new Pose2d(0, -27, Math.toRadians(270)), Math.toRadians(90), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-20, 20))
                    .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
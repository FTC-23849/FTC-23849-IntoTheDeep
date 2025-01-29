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

    public static final int PUSHING_VEL_ACC = 120;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 14)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(35, -60, Math.toRadians(270)))
                .strafeTo(new Vector2d(0, -35), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70))
//                .setTangent(159.6)
                .strafeTo(new Vector2d(35, -60), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70))

//                .setTangent(159.62)
                .strafeTo(new Vector2d(-2, -35), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70))
//                .setTangent(159.62)
                .strafeTo(new Vector2d(35, -60), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70))
//                .setTangent(15)
//                .splineToConstantHeading(new Vector2d(3, -30), Math.toRadians(90), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-30, 50))

//                .setTangent(159.6)
//                .lineToY(-35)
//                .setTangent(159.6)
//                .lineToY(-60)
//
//                .setTangent(159.62)
//                .lineToY(-35)
//                .setTangent(159.62)
//                .lineToY(-60)
//
//                .setTangent(159.64)
//                .lineToY(-35)
//                .setTangent(159.64)
//                .lineToY(-60)
//
//                .setTangent(159.66)
//                .lineToY(-35)
//                .setTangent(159.66)
//                .lineToY(-60)
//
//                .setTangent(159.68)
//                .lineToY(-35)
//                .setTangent(159.68)
//                .lineToY(-60)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
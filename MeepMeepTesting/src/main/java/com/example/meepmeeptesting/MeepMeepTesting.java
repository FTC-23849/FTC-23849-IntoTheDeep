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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(9, -61, Math.toRadians(90)))
                        .strafeTo(new Vector2d(6, -28))
                        .setTangent(Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(25, -40), Math.toRadians(10))
                                .splineToConstantHeading(new Vector2d(50, -9/*originally-10*/), Math.toRadians(0))
                                .setTangent(Math.toRadians(270))


//                        .splineToLinearHeading(new Pose2d(-22, -5, Math.toRadians(0)), Math.toRadians(-10))
//                        .setTangent(85)
//                        .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.toRadians(260))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
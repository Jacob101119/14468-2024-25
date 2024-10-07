package org.firstinspires.ftc.teamcode.autoMeet1;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.MecanumDrive;




    @Autonomous
    public final class SpecimenHighRung extends LinearOpMode {

        BaseRobot robot = new BaseRobot(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-12, -60, 90));
        @Override

        public void runOpMode() throws InterruptedException {


                waitForStart();
//new updates to run movements
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)



                                .splineTo(new Vector2d(-10.50, -40.00), Math.toRadians(90.00))//move to sub for high rung placement
                                //slides up, move forward a tiny bit, slides down a bit, back up, move back, slides down
                                //robot.setSlidesPos(robot.getSLIDES_TO_HIGH_RUNG())

                                .splineTo(new Vector2d(-14.08, -38.61), Math.toRadians(212.28))//turn to go grab 1st yellow sample
                                //pivot motor horizontal

                                .splineTo(new Vector2d(-49.21, -48), Math.toRadians(90.00))//lined up for first yellow sample
                                //grab, pivot motor vertical

                                .splineTo(new Vector2d(-50.00, -49.10), Math.toRadians(225.00))//after grabbing sample, turn


                                .splineTo(new Vector2d(-56.18, -55.78), Math.toRadians(224.91))//move to high bucket
                                //slides up, move forward, grasper open, move back, slides down

                                .build());

                /*

                //auto on human player wing side
                //start posnew Pose2d(12.00, -60.00, Math.toRadians(90.00)))

                    .splineTo(new Vector2d(10.73, -40.00), Math.toRadians(90.00)) //move to place preloaded specimen on high rung
                    //slides up, move forward a tiny bit, slides down a tiny bit, slides up, move back, slides down

                    .splineTo(new Vector2d(19.71, -40.00), Math.toRadians(0.00))

                    .splineTo(new Vector2d(35.53, -35.53), Math.toRadians(90.00))
                    .splineTo(new Vector2d(44.11, -14.88), Math.toRadians(-90.00))
                    .splineTo(new Vector2d(49.00, -14.21), Math.toRadians(-90.00))
                    
                    .splineTo(new Vector2d(49, -58, Math.toRadians(-90))


                    .build();


                 */





        }
    }


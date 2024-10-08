package org.firstinspires.ftc.teamcode.autoMeet1.RED.BucketSide;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.MecanumDrive;




    @Autonomous
    public final class RED_SP_3YHB extends LinearOpMode {

        BaseRobot robot;
        MecanumDrive drive;
        @Override

        public void runOpMode() throws InterruptedException {
                robot = new BaseRobot(hardwareMap);
                drive = new MecanumDrive(hardwareMap, new Pose2d(-60, -12, 0));

                waitForStart();
//new updates to run movements
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)

                                .strafeToLinearHeading(new Vector2d(-40, -12), 0)
                                //.strafeToLinearHeading(new Vector2d(-40, -12, 90))//move to sub for high rung placement /-10.5
                                //slides up, move forward a tiny bit, slides down a bit, back up, move back, slides down
                                //robot.setSlidesPos(robot.getSLIDES_TO_HIGH_RUNG())


                                .strafeToLinearHeading(new Vector2d(-38.61, -9.98), Math.toRadians(122.28))//turn to go grab 1st yellow sample
                                //pivot motor horizontal

                                .strafeToLinearHeading(new Vector2d(-48, 25.21), Math.toRadians(0))//lined up for first yellow sample
                                //grab, pivot motor vertical
                                //robot.setPivotMotorPos(5003)

                                .strafeToLinearHeading(new Vector2d(-49.10, 26), Math.toRadians(135))//after grabbing sample, turn


                                .strafeToLinearHeading(new Vector2d(-55.78,32.18), Math.toRadians(134.91))//move to high bucket
                                //slides up, move forward, grasper open, move back, slides down


                                .strafeToLinearHeading(new Vector2d(-48, 25.21), Math.toRadians(0))//lined up for second yellow sample

                                .strafeToLinearHeading(new Vector2d(-49.10, 26), Math.toRadians(135))//after grabbing sample, turn


                                .strafeToLinearHeading(new Vector2d(-55.78,32.18), Math.toRadians(134.91))//move to high bucket
                                //slides up, move forward, grasper open, move back, slides down

                                .strafeToLinearHeading(new Vector2d(-48, 25.21), Math.toRadians(0))//lined up for third yellow sample

                                .strafeToLinearHeading(new Vector2d(-49.10, 26), Math.toRadians(135))//after grabbing sample, turn


                                .strafeToLinearHeading(new Vector2d(-55.78,32.18), Math.toRadians(134.91))//move to high bucket
                                //slides up, move forward, grasper open, move back, slides down

                                .strafeToLinearHeading(new Vector2d(-60, -12), 0)


                                .build());



                /*

                //auto on human player wing side
                //start pos new Pose2d(12.00, -60.00, Math.toRadians(90.00)))

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


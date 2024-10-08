package org.firstinspires.ftc.teamcode.autoMeet1.RED.BucketSide;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.MecanumDrive;




@Autonomous
public final class redo extends LinearOpMode {

    BaseRobot robot;
    MecanumDrive drive;
    @Override


    public void runOpMode() throws InterruptedException {
        robot = new BaseRobot(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(-12.60, -62.48, 90));


        Action moveToSubAction = robot.drive.actionBuilder(drive.pose)

                .strafeToConstantHeading(new Vector2d(-11.00, -41.00))
                .build();


        waitForStart();
//new updates to run movements
        Actions.runBlocking(moveToSubAction);

        robot.setSlidesPos(817);
        //slides up

        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.pose)//move farther to high rung

                .strafeToConstantHeading(new Vector2d(-11, -36))
                .build());
        //----
        robot.setSlidesPos(790);
        robot.setGrasperPos(robot.getGRASPER_OPEN());
        robot.setSlidesPos(830);

        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.pose)

                .strafeToConstantHeading(new Vector2d(-11.00, -41.00))
                .build());

        robot.setSlidesPos(0);

        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.pose)

                        .strafeToConstantHeading(new Vector2d(-49, -45))//move to first yellow sample
                        .build());

        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_HORIZONTAL());
        robot.setSlidesPos(50);
        robot.setGrasperPos(robot.getGRASPER_CLOSED());
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_HORIZONTAL()-100);
        robot.setSlidesPos(0);
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL());


        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.pose)

                .strafeToLinearHeading(new Vector2d(-56, -56), 225)
                .build());


    }
}


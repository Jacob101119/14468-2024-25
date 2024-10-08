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
public final class USE_RED_SP_2YHB extends LinearOpMode {

    BaseRobot robot;
    MecanumDrive drive;
    @Override


    public void runOpMode() throws InterruptedException{
        robot = new BaseRobot(hardwareMap, new Pose2d(-12.60, -62.48, 90));

        robot.setGrasperPos(robot.getGRASPER_CLOSED());
        robot.updateGrasperPos();
        robot.setAxlePos(robot.getAXLE_SERVO_DOWN());
        robot.updateAxleServoPos();


        Action moveToSubAction = robot.drive.actionBuilder(robot.drive.pose)

                .strafeToConstantHeading(new Vector2d(-12.60, -41.00))
                .build();


        waitForStart();
//new updates to run movements
        //Actions.runBlocking(moveToSubAction);
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToLinearHeading(new Vector2d(-12.60, -41), 90)
                                .build());


        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL());
        robot.updatePivotMotorPos();
        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();
        robot.delay(2);
        //add delay
        robot.setSlidesPos(robot.getSLIDES_ABOVE_HIGH_RUNG());
        robot.updateSlidesPos();
        robot.delay(2);

        //slides up

        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.pose)//move farther to high rung

                .strafeToConstantHeading(new Vector2d(-12.60, -35.9))
                .build());
        //----
        robot.setSlidesPos(robot.getSLIDES_PUT_SP_ON_HIGH_RUNG());
        robot.updateSlidesPos();
        robot.delay(2);
        robot.setGrasperPos(robot.getGRASPER_OPEN());
        robot.updateGrasperPos();
        robot.delay(.2);
        robot.setSlidesPos(robot.getSLIDES_ABOVE_HIGH_RUNG());
        robot.updateSlidesPos();
        robot.delay(.3);
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.pose)//move back

                .strafeToConstantHeading(new Vector2d(-11.00, -41.00))
                .build());

        robot.setSlidesPos(0);
        robot.updateSlidesPos();
        robot.delay(.2);
        robot.setPivotMotorPos(0);
        robot.updatePivotMotorPos();
        robot.delay(4);

        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToConstantHeading(new Vector2d(-11, -48))
                        .strafeToConstantHeading(new Vector2d(-49, -45))//move to first yellow sample
                        .build());

        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_HORIZONTAL());
        robot.updatePivotMotorPos();
        robot.delay(.2);
        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();
        robot.delay(.2);
        robot.setGrasperPos(robot.getGRASPER_CLOSED());
        robot.updateGrasperPos();
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL());
        robot.updatePivotMotorPos();


        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.pose)

                .strafeToLinearHeading(new Vector2d(-56, -56), 135)//move to bucket
                .build());

        robot.setSlidesPos(robot.getSLIDES_MAX());
        //move forward?
        robot.setGrasperPos(robot.getGRASPER_OPEN());

        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.pose)

                .strafeToLinearHeading(new Vector2d(-59, -45), 90)//to second yellow sample
                .build());

        //grab second sample
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_HORIZONTAL());
        robot.setGrasperPos(robot.getGRASPER_CLOSED());
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL());


        //move to bucket
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.pose)

                .strafeToLinearHeading(new Vector2d(-56, -56), 225)//move to bucket
                .build());

        //score
        robot.setSlidesPos(robot.getSLIDES_MAX());
        robot.setGrasperPos(robot.getGRASPER_OPEN());


        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.pose)//move back a little bit

                .strafeToConstantHeading(new Vector2d(-54, -54))

                .build());

        robot.setSlidesPos(0);
        robot.setPivotMotorPos(0);

        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.pose)//move close to sub

                .strafeToLinearHeading(new Vector2d(-12, -47), 90)

                .build());



    }
}


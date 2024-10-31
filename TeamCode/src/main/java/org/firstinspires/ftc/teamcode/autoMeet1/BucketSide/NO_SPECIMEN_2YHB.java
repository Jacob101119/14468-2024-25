package org.firstinspires.ftc.teamcode.autoMeet1.BucketSide;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;




@Autonomous
public final class NO_SPECIMEN_2YHB extends LinearOpMode {

    BaseRobot robot;
    MecanumDrive drive;
    @Override


    public void runOpMode() throws InterruptedException{
        robot = new BaseRobot(hardwareMap, new Pose2d(-39, -62.8,Math.toRadians(180)));
        robot.setGrasperPos(robot.getGRASPER_CLOSED());
        robot.updateGrasperPos();
        robot.setAxlePos(robot.getAXLE_SERVO_DOWN());
        robot.updateAxleServoPos();
        robot.setGimbalPos(robot.getGIMBAL_RESTING_POS());
        robot.updateGimbalPos();
















        waitForStart();






        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL()-560);
        robot.updatePivotMotorPos();

        Action moveToHighBucket1 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(225))
                .build();
        //Actions.runBlocking(moveToHighBucket1);
        robot.delay(1.7);
        robot.setAxlePos(robot.getAXLE_SERVO_BACK());
        robot.updateAxleServoPos();
        robot.setSlidesPos(robot.getSLIDES_MAX());
        robot.updateSlidesPos();
        robot.delay(.2);
        robot.setAxlePos(robot.getAXLE_SERVO_BACK());
        robot.updateAxleServoPos();
        robot.delay(2);

        Action moveForwardAtHB1 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-46, -62.8))
                //.strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(225))
                .build();
        Actions.runBlocking(moveForwardAtHB1);
        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();
        //move forward?
        robot.delay(.4);
        robot.setGrasperPos(robot.getGRASPER_OPEN());
        robot.updateGrasperPos();
        robot.delay(.4);
        robot.setAxlePos(robot.getAXLE_SERVO_BACK());
        robot.updateAxleServoPos();

        robot.updateAll();
        robot.delay(.2);
        Action moveBackFromHB = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-40, -62.8))
                .build();
        Actions.runBlocking(moveBackFromHB);//move back from net zone a bit
        robot.delay(.2);


        robot.setSlidesPos(0);
        robot.updateSlidesPos();
        robot.delay(1.3);



        telemetry.addData("position", robot.drive.pose);
        telemetry.update();

        robot.setGrasperPos(robot.getGRASPER_OPEN());
        robot.updateGrasperPos();





        Action moveToFirstYellowSample = robot.drive.actionBuilder(robot.drive.pose)
                //.strafeToConstantHeading(new Vector2d(-14, -48))
                .strafeToLinearHeading(new Vector2d(-53.4, -54.6), Math.toRadians(90))//move to first yellow sample
                .build();
        Actions.runBlocking(moveToFirstYellowSample);
        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();

        robot.setPivotMotorPos(4660);
        robot.updatePivotMotorPos();
        robot.delay(.2);

        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();
        robot.delay(3);

        robot.setGrasperPos(robot.getGRASPER_CLOSED());
        robot.updateGrasperPos();
        robot.delay(.3);

        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL()-600);
        robot.updatePivotMotorPos();
        robot.delay(.8);

        Action moveToHighBucket2 = robot.drive.actionBuilder(robot.drive.pose)
                //.strafeToLinearHeading(new Vector2d(-48, -45), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-46.2, -46.2), Math.toRadians(225))
                .build();
        Actions.runBlocking(moveToHighBucket2);

        robot.setSlidesPos(robot.getSLIDES_MAX());
        robot.updateSlidesPos();
        robot.delay(.2);
        robot.setAxlePos(robot.getAXLE_SERVO_BACK());
        robot.updateAxleServoPos();
        robot.delay(2);

        Action moveForwardAtHB2 = robot.drive.actionBuilder(robot.drive.pose)
                //.strafeToLinearHeading(new Vector2d(-48, -45), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-50.5, -50.5), Math.toRadians(225))
                .build();
        Actions.runBlocking(moveForwardAtHB2);
        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();
        //move forward?
        robot.delay(1.5);
        robot.setGrasperPos(robot.getGRASPER_OPEN());
        robot.updateGrasperPos();
        robot.delay(.6);
        robot.setAxlePos(robot.getAXLE_SERVO_BACK());
        robot.updateAxleServoPos();
        Action moveBackFromHB2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-45, -45))
                .build();
        Actions.runBlocking(moveBackFromHB2);//move back from net zone a bit
        robot.delay(.2);

        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();
        robot.setSlidesPos(0);
        robot.updateSlidesPos();
        robot.delay(3);


        robot.resetAll();
        robot.updateAll();
        robot.delay(5);

    }
}


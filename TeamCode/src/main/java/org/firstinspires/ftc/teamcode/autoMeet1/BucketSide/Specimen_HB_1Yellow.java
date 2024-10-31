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
public final class Specimen_HB_1Yellow extends LinearOpMode {

    BaseRobot robot;
    MecanumDrive drive;
    @Override


    public void runOpMode() throws InterruptedException{
        robot = new BaseRobot(hardwareMap, new Pose2d(-12, -62.48,Math.toRadians(90)));
        robot.setGrasperPos(robot.getGRASPER_CLOSED());
        robot.updateGrasperPos();
        robot.setAxlePos(robot.getAXLE_SERVO_DOWN());
        robot.updateAxleServoPos();
        robot.setGimbalPos(robot.getGIMBAL_RESTING_POS());
        robot.updateGimbalPos();
















        waitForStart();



        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_TO_HIGH_CHAMBER());
        robot.updatePivotMotorPos();
        robot.delay(.6);
        robot.setSlidesPos(robot.getSLIDES_ABOVE_HIGH_RUNG());
        robot.updateSlidesPos();
        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();

        Action moveToSub = robot.drive.actionBuilder(robot.drive.pose)

                .strafeToConstantHeading(new Vector2d(-10, -42.8))
                .build();
        Actions.runBlocking(moveToSub);//drive forward

        telemetry.addData("pose", Math.toDegrees((robot.drive.pose.heading.toDouble())));
        telemetry.update();


        robot.delay(.3);

        robot.setSlidesPos(robot.getSLIDES_PUT_SP_ON_HIGH_RUNG());//clip specimen
        robot.updateSlidesPos();
        robot.delay(.5);


        robot.setGrasperPos(robot.getGRASPER_OPEN());//release specimen
        robot.updateGrasperPos();
        //robot.delay(.2);



        //moves sideways then back
        //TODO: fix the way it moves sideways to prevent getting stuck on specimen
        Action moveBackAwayFromSub = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-12, -40))
                .strafeToConstantHeading(new Vector2d(-13, -53.00))
                .build();
        Actions.runBlocking(moveBackAwayFromSub);
        robot.setSlidesPos(0);//slides down
        robot.updateSlidesPos();
        robot.delay(.7);



        Action moveToFirstYellowSample = robot.drive.actionBuilder(robot.drive.pose)
                //.strafeToConstantHeading(new Vector2d(-14, -48))
                .strafeToConstantHeading(new Vector2d(-53, -54.4))//move to first yellow sample
                .build();
        Actions.runBlocking(moveToFirstYellowSample);

        robot.setPivotMotorPos(5151);
        robot.updatePivotMotorPos();
        robot.delay(.2);

        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();
        robot.delay(3);

        robot.setGrasperPos(robot.getGRASPER_CLOSED());
        robot.updateGrasperPos();
        robot.delay(.3);

        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL()-180);
        robot.updatePivotMotorPos();
        robot.delay(.8);

        Action moveToHighBucket1 = robot.drive.actionBuilder(robot.drive.pose)
                //.strafeToLinearHeading(new Vector2d(-48, -45), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-46.2, -46.2), Math.toRadians(225))
                .build();
        Actions.runBlocking(moveToHighBucket1);

        robot.setSlidesPos(robot.getSLIDES_MAX());
        robot.updateSlidesPos();
        robot.delay(.2);
        robot.setAxlePos(robot.getAXLE_SERVO_BACK());
        robot.updateAxleServoPos();
        robot.delay(2);

        Action moveForwardAtHB1 = robot.drive.actionBuilder(robot.drive.pose)
                //.strafeToLinearHeading(new Vector2d(-48, -45), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(225))
                .build();
        Actions.runBlocking(moveForwardAtHB1);
        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();
        //move forward?
        robot.delay(1.5);
        robot.setGrasperPos(robot.getGRASPER_OPEN());
        robot.updateGrasperPos();
        robot.delay(.6);
        robot.setAxlePos(robot.getAXLE_SERVO_BACK());
        robot.updateAxleServoPos();
        Action moveBackFromHB = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-45, -45))
                .build();
        Actions.runBlocking(moveBackFromHB);//move back from net zone a bit
        robot.delay(.2);

        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();
        robot.setSlidesPos(0);
        robot.updateSlidesPos();
        robot.delay(3);


        /*
        Action moveToSecondYellowSample = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-54, -56), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-56, -45), Math.toRadians(90))
                .build();
        Actions.runBlocking(moveToSecondYellowSample);

        //grab second sample
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_HORIZONTAL());
        robot.updatePivotMotorPos();
        robot.delay(3);

        robot.setGrasperPos(robot.getGRASPER_CLOSED());
        robot.updateGrasperPos();
        robot.delay(.4);

        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL());
        robot.updatePivotMotorPos();
        robot.delay(2);


        Action moveToHighBucket2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-54, -45), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(225))
                .build();
        Actions.runBlocking(moveToHighBucket2);//bring yellow sample to HB

        robot.setSlidesPos(robot.getSLIDES_MAX());
        robot.updateSlidesPos();
        //move forward?
        robot.delay(3);
        robot.setGrasperPos(robot.getGRASPER_OPEN());
        robot.updateGrasperPos();
        robot.delay(.2);

        Action moveBackFromHB2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-50, -50))
                .build();
        Actions.runBlocking(moveBackFromHB);//move back from net zone a bit
        robot.delay(.2);
        */




        robot.resetAll();
        //servos
        robot.updateAxleServoPos();
        robot.updateGimbalPos();
        robot.updateGrasperPos();
        //motors
        robot.updatePivotMotorPos();
        robot.updateSlidesPos();
        robot.delay(5);

    }
}


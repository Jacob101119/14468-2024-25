package org.firstinspires.ftc.teamcode.autoMeet1.OZSide;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;




@Autonomous
public final class RED_BLUE_3SP_V3 extends LinearOpMode {

    BaseRobot robot;
    MecanumDrive drive;
    @Override


    public void runOpMode() throws InterruptedException{

        robot = new BaseRobot(hardwareMap, new Pose2d(12, -62.48, Math.toRadians(90)));


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

                .strafeToConstantHeading(new Vector2d(8, -44))
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
                .strafeToConstantHeading(new Vector2d(4, -43.8))
                .strafeToConstantHeading(new Vector2d(2, -53.00))
                .build();
        Actions.runBlocking(moveBackAwayFromSub);


        //slides down



        //second specimen
        //_____________________________________________________________________________________________________

        //move to observation zone



        robot.setAxlePos(robot.getAXLE_SERVO_GRAB_FROM_WALL());
        robot.updateAxleServoPos();
        robot.setSlidesPos(0);//slides down
        robot.updateSlidesPos();
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_GRAB_FROM_WALL()+100);
        robot.updatePivotMotorPos();



        //second specimen
        //_____________________________________________________________________________________________________

        //move to observation zone



        Action updatedMoveToOZ = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(47, -38), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(47, -43.6), Math.toRadians(-90))
                .build();
        Actions.runBlocking(updatedMoveToOZ);
        robot.delay(.15);
        //grab specimen, pivot motor up
        robot.setGrasperPos(robot.getGRASPER_CLOSED());
        robot.updateGrasperPos();
        robot.delay(.2);
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL());
        robot.updatePivotMotorPos();
        robot.delay(.2);//TODO: see if this is too little




        Action moveToSub2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(50.95, -41.00), Math.toRadians(-90.00))
                .strafeToLinearHeading(new Vector2d(30.17, -49.34), Math.toRadians(180.00))
                .strafeToLinearHeading(new Vector2d(9, -55), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(6, -46.4), Math.toRadians(90.00))
                .build();
        Action updatedMoveToSub2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(2, -46.4), Math.toRadians(90))
                .build();
        Actions.runBlocking(updatedMoveToSub2);//TODO: see if new path is ok
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_TO_HIGH_CHAMBER());
        robot.updatePivotMotorPos();

        robot.delay(.3);//TODO: see if this too low


        robot.setSlidesPos(robot.getSLIDES_ABOVE_HIGH_RUNG());
        robot.updateSlidesPos();
        robot.delay(.4);

        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_TO_HIGH_CHAMBER()+100);
        robot.updatePivotMotorPos();
        robot.delay(.5);

        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();
        robot.delay(.8);

        robot.setSlidesPos(robot.getSLIDES_PUT_SP_ON_HIGH_RUNG());//clip specimen
        robot.updateSlidesPos();
        robot.delay(.4);

        robot.setGrasperPos(robot.getGRASPER_OPEN());
        robot.updateGrasperPos();
        //robot.delay(.2);



        Action moveBackAwayFromSub2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-4, -46.4))
                .strafeToConstantHeading(new Vector2d(-6, -53))
                .build();
        Actions.runBlocking(moveBackAwayFromSub2);

        robot.setPivotMotorPos(4500);
        robot.updatePivotMotorPos();
        robot.delay(.2);

        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();
        //robot.delay(3);



        Action moveToFirstRedSample = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(14, -53))
                .strafeToConstantHeading(new Vector2d(46.8, -48))//move to first yellow sample
                .build();
        Actions.runBlocking(moveToFirstRedSample);

        robot.setPivotMotorPos(5151);
        robot.updatePivotMotorPos();
        robot.delay(2);


        robot.setGrasperPos(robot.getGRASPER_CLOSED());
        robot.updateGrasperPos();
        robot.delay(.3);

        robot.setPivotMotorPos(0);
        robot.updatePivotMotorPos();

        //robot.setAxlePos(robot.getAXLE_SERVO_BACK());
        //robot.updateAxleServoPos();

        //robot.delay(.8);

        Action bringRed1ToOZ = robot.drive.actionBuilder(robot.drive.pose)
                //.strafeToLinearHeading(new Vector2d(47.8, -36), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(46.8, -53.8), Math.toRadians(90))
                .build();
        Actions.runBlocking(bringRed1ToOZ);
        robot.delay(.2);

        robot.setAxlePos(robot.getAXLE_SERVO_DOWN());
        robot.updateAxleServoPos();
        robot.delay(.8);
        //robot.setGrasperPos(robot.getGRASPER_OPEN());
        //robot.updateGrasperPos();
        robot.delay(.2);




        robot.resetAll();
        //robot.setAxlePos(robot.getAXLE_SERVO_BACK());
        robot.delay(4);

        /*
        //move forward for human player to grab sample then move back again
        Action moveForward2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(50, -40))
                .build();
        robot.delay(.5);
        Action park = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(50, -51))
                .build();
        Actions.runBlocking(park);


         */




    }
}


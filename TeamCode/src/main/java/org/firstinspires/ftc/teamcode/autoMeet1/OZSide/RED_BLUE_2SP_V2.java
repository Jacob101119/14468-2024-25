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
public final class RED_BLUE_2SP_V2 extends LinearOpMode {

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
//new updates to run movements

        robot.setSlidesPos(robot.getSLIDES_ABOVE_HIGH_RUNG());
        robot.updateSlidesPos();//TODO: check if this goes out of sizing
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_TO_HIGH_CHAMBER());
        robot.updatePivotMotorPos();

        Action moveToSub = robot.drive.actionBuilder(robot.drive.pose)

                .strafeToConstantHeading(new Vector2d(9, -43.5))
                .build();
        Actions.runBlocking(moveToSub);//drive forward


        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();
        robot.delay(.3);

        robot.setSlidesPos(robot.getSLIDES_PUT_SP_ON_HIGH_RUNG());//clip specimen
        robot.updateSlidesPos();
        robot.delay(.5);


        robot.setGrasperPos(robot.getGRASPER_OPEN());//release specimen
        robot.updateGrasperPos();
        robot.delay(.3);


        //moves sideways then back
        //TODO: fix the way it moves sideways to prevent getting stuck on specimen
        Action moveBackAwayFromSub = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(7, -43.5))
                .strafeToConstantHeading(new Vector2d(9, -53.00))
                .build();
        Actions.runBlocking(moveBackAwayFromSub);



        //slides down


        robot.setAxlePos(robot.getAXLE_SERVO_GRAB_FROM_WALL());
        robot.updateAxleServoPos();
        robot.setSlidesPos(0);//slides down
        robot.updateSlidesPos();
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_GRAB_FROM_WALL());
        robot.updatePivotMotorPos();



        //second specimen
        //_____________________________________________________________________________________________________

        //move to observation zone

        Action MoveToOZ = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(18.00, -43.5), Math.toRadians(50.00))
                .strafeToLinearHeading(new Vector2d(31.11, -40), Math.toRadians(10.00))
                .strafeToLinearHeading(new Vector2d(49.4, -40), Math.toRadians(-90.00))
                .strafeToLinearHeading(new Vector2d(49.4, -43), Math.toRadians(-90.00))
                .build();
        Action updatedMoveToOZ = robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToLinearHeading(new Vector2d(49.4, -40), Math.toRadians(-90))
                                .strafeToLinearHeading(new Vector2d(49.4, -43), Math.toRadians(-90))
                                        .build();
        Actions.runBlocking(updatedMoveToOZ);//TODO: see if this new movement works without running into stuff
        robot.delay(.2);
        //grab specimen, pivot motor up
        robot.setGrasperPos(robot.getGRASPER_CLOSED());
        robot.updateGrasperPos();
        robot.delay(.3);
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL());
        robot.updatePivotMotorPos();
        robot.delay(.3);//TODO: see if this is too little




        Action moveToSub2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(50.95, -41.00), Math.toRadians(-90.00))
                .strafeToLinearHeading(new Vector2d(30.17, -49.34), Math.toRadians(180.00))
                .strafeToLinearHeading(new Vector2d(9, -55), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(6, -45.5), Math.toRadians(90.00))
                .build();
        Action updatedMoveToSub2 = robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToLinearHeading(new Vector2d(6, -45.5), Math.toRadians(90))
                                .build();
        Actions.runBlocking(updatedMoveToSub2);//TODO: see if new path is ok
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_TO_HIGH_CHAMBER());
        robot.updatePivotMotorPos();

        robot.delay(.3);//TODO: see if this too low


        robot.setSlidesPos(robot.getSLIDES_ABOVE_HIGH_RUNG()-50);
        robot.updateSlidesPos();
        robot.delay(.7);

        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();
        robot.delay(.5);

        robot.setSlidesPos(robot.getSLIDES_PUT_SP_ON_HIGH_RUNG());//clip specimen
        robot.updateSlidesPos();
        robot.delay(.5);

        robot.setGrasperPos(robot.getGRASPER_OPEN());
        robot.updateGrasperPos();
        robot.delay(.3);



        Action moveBackAwayFromSub2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(7, -41.00))
                .build();
        Actions.runBlocking(moveBackAwayFromSub2);


        robot.resetAll();
        robot.delay(5);

        /*robot.setPivotMotorPos(robot.getPIVOT_MOTOR_HORIZONTAL()-500);
        robot.updatePivotMotorPos();
        robot.delay(2);




        //third specimen
        //________________________________________________________________________________________________
        //Actions.runBlocking(moveToOZ);
        Action MoveToOZ2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(18.00, -54.00), Math.toRadians(50.00))
                .strafeToLinearHeading(new Vector2d(31.11, -50.15), Math.toRadians(10.00))
                .strafeToLinearHeading(new Vector2d(50.95, -48.13), Math.toRadians(-90.00))
                .build();
        Actions.runBlocking(MoveToOZ2);
        robot.delay(.5);
        //grab specimen, pivot motor up
        robot.setGrasperPos(robot.getGRASPER_CLOSED());
        robot.updateGrasperPos();
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL());
        robot.delay(1);
        //robot.delay(.5);

        Action moveToSub3 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(50.95, -41.00), Math.toRadians(-90.00))
                .strafeToLinearHeading(new Vector2d(30.17, -49.34), Math.toRadians(180.00))
                .strafeToLinearHeading(new Vector2d(8.8, -43.50), Math.toRadians(90.00))
                .build();
        Actions.runBlocking(moveToSub3);

        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL()+260);
        robot.updatePivotMotorPos();

        robot.delay(.1);

        //robot.delay(1);
        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();
        robot.delay(1.3);
        //add delay
        robot.setSlidesPos(robot.getSLIDES_ABOVE_HIGH_RUNG());
        robot.updateSlidesPos();
        robot.delay(1);

        robot.setSlidesPos(robot.getSLIDES_PUT_SP_ON_HIGH_RUNG());//clip specimen
        robot.updateSlidesPos();
        robot.delay(1);




        Action moveBackAwayFromSub3 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(12.00, -41.00))
                .build();
        // end 3rd specimen_________________________________________________________________________
        Actions.runBlocking(moveBackAwayFromSub3);

         */

        robot.resetAll();//reset method also updates all

        Action park = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(48, -51))
                .build();
        Actions.runBlocking(park);





    }
}


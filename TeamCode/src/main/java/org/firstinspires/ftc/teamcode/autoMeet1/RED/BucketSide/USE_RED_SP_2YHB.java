package org.firstinspires.ftc.teamcode.autoMeet1.RED.BucketSide;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;




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

        Action moveSlidesOverHighRung = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-12.60, -36))
                .build();

        Action moveBackAwayFromSub = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-11.00, -41.00))
                .build();

        Action moveToFirstYellowSample = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-11, -48))
                .strafeToConstantHeading(new Vector2d(-49, -45))//move to first yellow sample
                .build();

        Action moveToHighBucket1 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-49, -45), -90)
                .strafeToLinearHeading(new Vector2d(-56, -56), 225)
                .build();

        Action moveToSecondYellowSample = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-56, -56), 90)
                .strafeToLinearHeading(new Vector2d(-59.5, -45), 90)
                .build();

        Action moveToHighBucket2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-59.5, -45), -90)
                .strafeToLinearHeading(new Vector2d(-56, -56), 225)
                .build();

        Action moveBackFromHB = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-54, -54))
                .build();

        waitForStart();




        Actions.runBlocking(moveToSubAction);

        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL());
        robot.updatePivotMotorPos();
        robot.delay(.1);
        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();
        robot.delay(2);
        //add delay
        robot.setSlidesPos(robot.getSLIDES_ABOVE_HIGH_RUNG());
        robot.updateSlidesPos();
        robot.delay(2);

        //slides up

        Actions.runBlocking(moveSlidesOverHighRung);
        robot.delay(.5);
        //----
        robot.setSlidesPos(robot.getSLIDES_PUT_SP_ON_HIGH_RUNG()-40);//clip specimen
        robot.updateSlidesPos();
        robot.delay(2);

        robot.setGrasperPos(robot.getGRASPER_OPEN());//release specimen
        robot.updateGrasperPos();
        robot.delay(2);

        //robot.setSlidesPos(robot.getSLIDES_ABOVE_HIGH_RUNG());//slides back up
        //robot.updateSlidesPos();
        //robot.delay();

        Actions.runBlocking(moveBackAwayFromSub);

        robot.setSlidesPos(0);//slides down
        robot.updateSlidesPos();
        robot.delay(.2);

        //*note* delete these 3 lines once specimen works
        robot.setPivotMotorPos(0);//pivot motor back
        robot.updatePivotMotorPos();
        robot.delay(4);
        //end note

        Actions.runBlocking(moveToFirstYellowSample);

        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_HORIZONTAL());
        robot.updatePivotMotorPos();
        robot.delay(.2);

        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();
        robot.delay(.2);

        robot.setGrasperPos(robot.getGRASPER_CLOSED());
        robot.updateGrasperPos();
        robot.delay(.2);

        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL());
        robot.updatePivotMotorPos();
        robot.delay(.5);


        Actions.runBlocking(moveToHighBucket1);

        robot.setSlidesPos(robot.getSLIDES_MAX());
        robot.updateSlidesPos();

        //move forward?
        robot.delay(1);
        robot.setGrasperPos(robot.getGRASPER_OPEN());
        robot.updateGrasperPos();
        robot.delay(.2);

        Actions.runBlocking(moveBackFromHB);//move back from net zone a bit
        robot.delay(.2);

        robot.setSlidesPos(0);
        robot.updateSlidesPos();
        robot.delay(.3);


        Actions.runBlocking(moveToSecondYellowSample);

        //grab second sample
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_HORIZONTAL());
        robot.updatePivotMotorPos();
        robot.delay(1);

        robot.setGrasperPos(robot.getGRASPER_CLOSED());
        robot.updateGrasperPos();
        robot.delay(.4);

        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL());
        robot.updatePivotMotorPos();
        robot.delay(.5);


        //move to bucket
        Actions.runBlocking(moveToHighBucket2);//bring yellow sample to HB

        robot.setSlidesPos(robot.getSLIDES_MAX());
        robot.updateSlidesPos();
        //move forward?
        robot.delay(1);
        robot.setGrasperPos(robot.getGRASPER_OPEN());
        robot.updateGrasperPos();
        robot.delay(.2);

        Actions.runBlocking(moveBackFromHB);//move back from net zone a bit
        robot.delay(.2);

        robot.setSlidesPos(0);
        robot.updateSlidesPos();
        robot.delay(.3);


        Actions.runBlocking(moveBackFromHB);//move back from net zone a bit
        robot.delay(.5);


        robot.resetAll();
        //servos
        robot.updateAxleServoPos();
        robot.updateGimbalPos();
        robot.updateGrasperPos();
        //motors
        robot.updatePivotMotorPos();
        robot.updateSlidesPos();


    }
}


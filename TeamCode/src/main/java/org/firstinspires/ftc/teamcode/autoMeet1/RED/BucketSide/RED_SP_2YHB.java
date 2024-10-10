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
public final class RED_SP_2YHB extends LinearOpMode {

    BaseRobot robot;
    MecanumDrive drive;
    @Override


    public void runOpMode() throws InterruptedException{
        robot = new BaseRobot(hardwareMap, new Pose2d(-12, -62.48,Math.toRadians(90)));

        robot.setGrasperPos(robot.getGRASPER_CLOSED());
        robot.updateGrasperPos();
        robot.setAxlePos(robot.getAXLE_SERVO_DOWN());
        robot.updateAxleServoPos();


        Action moveToSubAction = robot.drive.actionBuilder(robot.drive.pose)

                .strafeToConstantHeading(new Vector2d(-12.5, -43.5))
                .build();






        waitForStart();




        Actions.runBlocking(moveToSubAction);

        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL()+300);
        robot.updatePivotMotorPos();

        robot.delay(.1);
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL());
        //robot.delay(1);
        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();
        robot.delay(2);
        //add delay
        robot.setSlidesPos(robot.getSLIDES_ABOVE_HIGH_RUNG());
        robot.updateSlidesPos();
        robot.delay(3);

        //slides up


        Action moveSlidesOverHighRung = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-12.60, -36), Math.toRadians(90))
                .build();


        //Actions.runBlocking(moveSlidesOverHighRung);
        robot.delay(.5);
        //----
        robot.setSlidesPos(robot.getSLIDES_PUT_SP_ON_HIGH_RUNG()-30);//clip specimen
        robot.updateSlidesPos();
        robot.delay(.4);

        Action moveAnInch = robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToConstantHeading(new Vector2d(-12.5, -39.5))
                                .build();
        Actions.runBlocking(moveAnInch);
        robot.delay(.2);
        robot.setSlidesPos(robot.getSLIDES_PUT_SP_ON_HIGH_RUNG()-60);//clip specimen
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL()+500);
        robot.updateSlidesPos();
        robot.updatePivotMotorPos();
        robot.delay(1);
        robot.setGrasperPos(robot.getGRASPER_OPEN());//release specimen
        robot.updateGrasperPos();
        robot.delay(1);

        //robot.setSlidesPos(robot.getSLIDES_ABOVE_HIGH_RUNG());//slides back up
        //robot.updateSlidesPos();
        //robot.delay();



        Action moveBackAwayFromSub = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-12.00, -41.00))
                .build();

        Actions.runBlocking(moveBackAwayFromSub);

        robot.setSlidesPos(0);//slides down
        robot.updateSlidesPos();
        robot.delay(1);

        //*note* delete these 3 lines once specimen works
        //robot.setPivotMotorPos(0);//pivot motor back
        //robot.updatePivotMotorPos();
        //end note

        Action moveToFirstYellowSample = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-14, -48))
                .strafeToConstantHeading(new Vector2d(-53.8, -54))//move to first yellow sample
                .build();
        Actions.runBlocking(moveToFirstYellowSample);

        robot.setPivotMotorPos(5221);
        robot.updatePivotMotorPos();
        robot.delay(.2);

        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();
        robot.delay(6);

        robot.setGrasperPos(robot.getGRASPER_CLOSED());
        robot.updateGrasperPos();
        robot.delay(.5);

        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL());
        robot.updatePivotMotorPos();
        robot.delay(.5);

        Action moveToHighBucket1 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-48, -45), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-51, -51), Math.toRadians(225))
                .build();
        Actions.runBlocking(moveToHighBucket1);

        robot.setSlidesPos(robot.getSLIDES_MAX());
        robot.updateSlidesPos();

        Action moveForwardAtHB1 = robot.drive.actionBuilder(robot.drive.pose)
                //.strafeToLinearHeading(new Vector2d(-48, -45), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(225))
                .build();
        Actions.runBlocking(moveForwardAtHB1);
        //move forward?
        robot.delay(3);
        robot.setGrasperPos(robot.getGRASPER_OPEN());
        robot.updateGrasperPos();
        robot.delay(.6);
        robot.setAxlePos(robot.getAXLE_SERVO_BACK());
        robot.updateAxleServoPos();
        Action moveBackFromHB = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-56, -56))
                .build();
        Actions.runBlocking(moveBackFromHB);//move back from net zone a bit
        robot.delay(.2);

        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();
        robot.setSlidesPos(0);
        robot.updateSlidesPos();
        robot.delay(3);

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


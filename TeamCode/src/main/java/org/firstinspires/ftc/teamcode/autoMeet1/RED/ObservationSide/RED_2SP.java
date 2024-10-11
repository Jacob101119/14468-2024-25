package org.firstinspires.ftc.teamcode.autoMeet1.RED.ObservationSide;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;




@Autonomous
public final class RED_2SP extends LinearOpMode {

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


        Action moveToSub = robot.drive.actionBuilder(robot.drive.pose)

                .strafeToConstantHeading(new Vector2d(9.5, -43.5))
                .build();













        waitForStart();
//new updates to run movements


        Actions.runBlocking(moveToSub);//drive forward

        //pivot up, sllides up, axle up
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

        robot.setSlidesPos(robot.getSLIDES_PUT_SP_ON_HIGH_RUNG()-200);//clip specimen
        robot.updateSlidesPos();
        robot.delay(1);

        //retry hang in case it doesn't clip the first time
        Action moveAnInch = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(9.5, -39.5))
                .build();
        Actions.runBlocking(moveAnInch);

        robot.delay(.2);
        robot.setSlidesPos(robot.getSLIDES_PUT_SP_ON_HIGH_RUNG()-300);//clip specimen
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL()+400);
        robot.updateSlidesPos();
        robot.updatePivotMotorPos();
        robot.delay(1.3);
        robot.setGrasperPos(robot.getGRASPER_OPEN());//release specimen
        robot.updateGrasperPos();
        robot.delay(.3);



        Action moveBackAwayFromSub = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(9.5, -53.00))
                .build();
        Actions.runBlocking(moveBackAwayFromSub);
        //slides down
        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();
        robot.setSlidesPos(0);//slides down
        robot.updateSlidesPos();
        robot.delay(.7);

        //pivot down for grabbing from OZ
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_HORIZONTAL()-500);
        robot.updatePivotMotorPos();
        robot.delay(2);


        //second specimen
        //________________________________________________________________

        //move to observation zone
        Action moveToOZ = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(40, -40), Math.toRadians(-180))
                .strafeToLinearHeading(new Vector2d(53, -42),Math.toRadians(-90))
                .build();
        //Actions.runBlocking(moveToOZ);
        Action updatedMoveToOZ = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(18.00, -54.00), Math.toRadians(50.00))
                .strafeToLinearHeading(new Vector2d(31.11, -50.15), Math.toRadians(10.00))
                .strafeToLinearHeading(new Vector2d(50.95, -48.13), Math.toRadians(-90.00))
                .build();
        Actions.runBlocking(updatedMoveToOZ);
        robot.delay(.5);
        //grab specimen, pivot motor up
        robot.setGrasperPos(robot.getGRASPER_CLOSED());
        robot.updateGrasperPos();
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL());
        robot.delay(1);
        //robot.delay(.5);

        Action moveToSub2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(50.95, -41.00), Math.toRadians(-90.00))
                .strafeToLinearHeading(new Vector2d(30.17, -49.34), Math.toRadians(180.00))
                .strafeToLinearHeading(new Vector2d(8.8, -43.50), Math.toRadians(90.00))
                .build();
        Actions.runBlocking(moveToSub2);

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

        robot.setSlidesPos(robot.getSLIDES_PUT_SP_ON_HIGH_RUNG()-200);//clip specimen
        robot.updateSlidesPos();
        robot.delay(1);


        Action moveAnInch2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(9.5, -39.5))
                .build();
        Actions.runBlocking(moveAnInch2);
        robot.delay(.2);
        robot.setSlidesPos(robot.getSLIDES_PUT_SP_ON_HIGH_RUNG()-300);//clip specimen
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL()+400);
        robot.updateSlidesPos();
        robot.updatePivotMotorPos();
        robot.delay(1.3);
        robot.setGrasperPos(robot.getGRASPER_OPEN());//release specimen
        robot.updateGrasperPos();
        robot.delay(.3);

        Action moveBackAwayFromSub2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(12.00, -41.00))
                .build();

        Actions.runBlocking(moveBackAwayFromSub2);


        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_HORIZONTAL()-500);
        robot.updatePivotMotorPos();
        robot.delay(2);
        //move to observation zone



        //third specimen
        //__________________________________________________
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

        robot.setSlidesPos(robot.getSLIDES_PUT_SP_ON_HIGH_RUNG()-200);//clip specimen
        robot.updateSlidesPos();
        robot.delay(1);


        Action moveAnInch3 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(9.5, -39.5))
                .build();
        Actions.runBlocking(moveAnInch3);
        robot.delay(.2);
        robot.setSlidesPos(robot.getSLIDES_PUT_SP_ON_HIGH_RUNG()-300);//clip specimen
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL()+400);
        robot.updateSlidesPos();
        robot.updatePivotMotorPos();
        robot.delay(1.3);
        robot.setGrasperPos(robot.getGRASPER_OPEN());//release specimen
        robot.updateGrasperPos();
        robot.delay(.3);

        Action moveBackAwayFromSub3 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(12.00, -41.00))
                .build();

        Actions.runBlocking(moveBackAwayFromSub3);

        robot.resetAll();//reset method also updates all




    }
}


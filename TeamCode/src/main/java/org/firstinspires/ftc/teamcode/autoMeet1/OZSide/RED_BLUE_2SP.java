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
public final class RED_BLUE_2SP extends LinearOpMode {

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


        robot.delay(.3);

        Action moveToSub = robot.drive.actionBuilder(robot.drive.pose)

                .strafeToConstantHeading(new Vector2d(9, -43.5))
                .build();
        Actions.runBlocking(moveToSub);//drive forward


        //pivot up, slides up, axle up
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
        robot.delay(1.5);




        robot.setGrasperPos(robot.getGRASPER_OPEN());//release specimen
        robot.updateGrasperPos();
        robot.delay(.3);



        Action moveBackAwayFromSub = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(7, -43.5))
                .strafeToConstantHeading(new Vector2d(9, -53.00))
                .build();
        Actions.runBlocking(moveBackAwayFromSub);
        //slides down
        robot.setAxlePos(.38);
        robot.updateAxleServoPos();
        robot.setSlidesPos(0);//slides down
        robot.updateSlidesPos();
        robot.delay(.7);

        //pivot down for grabbing from OZ
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_HORIZONTAL()+260);
        robot.updatePivotMotorPos();
        robot.delay(2);


        //second specimen
        //_____________________________________________________________________________________________________

        //move to observation zone
        Action moveToOZ = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(40, -40), Math.toRadians(-180))
                .strafeToLinearHeading(new Vector2d(53, -42),Math.toRadians(-90))
                .build();
        //Actions.runBlocking(moveToOZ);
        Action updatedMoveToOZ = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(18.00, -40), Math.toRadians(50.00))
                .strafeToLinearHeading(new Vector2d(31.11, -40), Math.toRadians(10.00))
                .strafeToLinearHeading(new Vector2d(49.4, -40), Math.toRadians(-90.00))
                .strafeToLinearHeading(new Vector2d(49.4, -43), Math.toRadians(-90.00))
                .build();
        Actions.runBlocking(updatedMoveToOZ);
        robot.delay(2);
        //grab specimen, pivot motor up
        robot.setGrasperPos(robot.getGRASPER_CLOSED());
        robot.updateGrasperPos();
        robot.delay(1);
        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL());
        robot.updatePivotMotorPos();
        robot.delay(.4);


        /*Action moveBackFromOZ = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(48, -35))
                        .build();
        Actions.runBlocking(moveBackFromOZ);
        robot.delay(.1);
        //robot.resetAll();
        //robot.updateAll();
        //robot.delay(3);//get rid of this


        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL());
        robot.updatePivotMotorPos();
        robot.delay(1);

         */
        //robot.delay(.5);

        Action moveToSub2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(50.95, -41.00), Math.toRadians(-90.00))
                .strafeToLinearHeading(new Vector2d(30.17, -49.34), Math.toRadians(180.00))
                .strafeToLinearHeading(new Vector2d(9, -55), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(6, -45.5), Math.toRadians(90.00))
                .build();
        Actions.runBlocking(moveToSub2);

        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL()+300);
        robot.updatePivotMotorPos();

        robot.delay(1.3);

        //robot.delay(1);

        //add delay
        robot.setSlidesPos(robot.getSLIDES_ABOVE_HIGH_RUNG()-50);
        robot.updateSlidesPos();
        robot.delay(1);
        robot.setAxlePos(robot.getAXLE_SERVO_UP());
        robot.updateAxleServoPos();
        robot.delay(.5);

        robot.setSlidesPos(robot.getSLIDES_PUT_SP_ON_HIGH_RUNG());//clip specimen
        robot.updateSlidesPos();
        robot.delay(1);




        Action moveBackAwayFromSub2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(12.00, -41.00))
                .build();

        Actions.runBlocking(moveBackAwayFromSub2);


        robot.resetAll();
        robot.updateAll();
        robot.delay(5);

        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_HORIZONTAL()-500);
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

        robot.resetAll();//reset method also updates all

        Action park = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(48, -51))
                .build();
        Actions.runBlocking(park);





    }
}


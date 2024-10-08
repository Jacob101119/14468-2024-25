package org.firstinspires.ftc.teamcode.autoMeet1.RED.ObservationSide;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.MecanumDrive;




@Autonomous
public final class RED_SP_2R_OZ extends LinearOpMode {

    BaseRobot robot;
    MecanumDrive drive;
    @Override


    public void runOpMode() throws InterruptedException{
        robot = new BaseRobot(hardwareMap, new Pose2d(12, -62.48, 90));

        robot.setGrasperPos(robot.getGRASPER_CLOSED());
        robot.updateGrasperPos();
        robot.setAxlePos(robot.getAXLE_SERVO_DOWN());
        robot.updateAxleServoPos();
        robot.setGimbalPos(robot.getGIMBAL_RESTING_POS());
        robot.updateGimbalPos();


        Action moveToSub = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(12, -41.00))
                .build();

        Action slidesAboveSub = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(12, -37))
                .build();

        Action moveBackAwayFromSub = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(12, -43))
                .build();

        Action moveToFirstRedSample = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(36.5, -36))
                .strafeToLinearHeading(new Vector2d(47, -12), 0)
                .strafeToLinearHeading(new Vector2d(49, -12), -90)
                .build();

        Action moveFirstSampleToOZ = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(49, -56.5))
                .build();

        Action moveToSecondSample = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(49, -12))
                .strafeToConstantHeading(new Vector2d(58.5, -12))
                .build();

        Action moveSecondSampleToOZ = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(58.5, -56.5))
                .build();

        Action turnAround = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(55, -54))
                .strafeToLinearHeading(new Vector2d(55, -54), 130)
                .build();


        waitForStart();
//new updates to run movements


        Actions.runBlocking(moveToSub);

        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL());//set pivot motor to vertical
        robot.updatePivotMotorPos();

        robot.setSlidesPos(robot.getSLIDES_ABOVE_HIGH_RUNG());//slides above high rung
        robot.updateSlidesPos();

        robot.setAxlePos(robot.getAXLE_SERVO_UP());//axle servo up parallel to sub
        robot.updateAxleServoPos();

        Actions.runBlocking(slidesAboveSub);//drive forward a bit
        robot.delay(.3);

        robot.setSlidesPos(robot.getSLIDES_PUT_SP_ON_HIGH_RUNG());//slides down to clip specimen
        robot.updateSlidesPos();
        robot.delay(.3);

        robot.setGrasperPos(robot.getGRASPER_OPEN());//release specimen
        robot.updateGrasperPos();
        robot.delay(.3);

        robot.setSlidesPos(robot.getSLIDES_ABOVE_HIGH_RUNG());//slides back up
        robot.updateSlidesPos();
        robot.delay(.2);

        Actions.runBlocking(moveBackAwayFromSub);
        robot.delay(.2);

        robot.setAxlePos(robot.getAXLE_SERVO_DOWN());//move axle servo back in towards robot
        robot.updateAxleServoPos();

        robot.setSlidesPos(0);//slides down
        robot.updateSlidesPos();
        robot.delay(.2);

        robot.setPivotMotorPos(0);//bring pivot motor back
        robot.updatePivotMotorPos();
        robot.delay(.2);

        Actions.runBlocking(moveToFirstRedSample);//drive to first red sample on ground
        robot.delay(.2);

        Actions.runBlocking(moveFirstSampleToOZ);//push first red sample to observation zone
        robot.delay(.2);

        Actions.runBlocking(moveToSecondSample);//move to second red sample
        robot.delay(.2);

        Actions.runBlocking(moveSecondSampleToOZ);
        robot.delay(.2);

        Actions.runBlocking();



    }
}


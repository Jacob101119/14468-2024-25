package org.firstinspires.ftc.teamcode.autoMeet1.BLUE.ObservationSide;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;




@Autonomous
public final class BLUE_SP_2B_OZ extends LinearOpMode {

    BaseRobot robot;
    MecanumDrive drive;
    @Override


    public void runOpMode() throws InterruptedException{
        robot = new BaseRobot(hardwareMap, new Pose2d(-12, 62.48, Math.toRadians(-90)));

        robot.setGrasperPos(robot.getGRASPER_CLOSED());
        robot.updateGrasperPos();
        robot.setAxlePos(robot.getAXLE_SERVO_DOWN());
        robot.updateAxleServoPos();
        robot.setGimbalPos(robot.getGIMBAL_RESTING_POS());
        robot.updateGimbalPos();















        waitForStart();
//new updates to run movements


        Action moveToSub = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-12, 39.3))
                .build();
        Actions.runBlocking(moveToSub);


        robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL());//set pivot motor to vertical
        robot.updatePivotMotorPos();
        robot.delay(4);

        robot.setSlidesPos(robot.getSLIDES_ABOVE_HIGH_RUNG());//slides above high rung
        robot.updateSlidesPos();
        robot.delay(2);

        robot.setAxlePos(robot.getAXLE_SERVO_UP());//axle servo up parallel to sub
        robot.updateAxleServoPos();


        Action slidesAboveSub = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-12, 37))
                .build();
        //Actions.runBlocking(slidesAboveSub);//drive forward a bit
        robot.delay(.3);

        robot.setSlidesPos(robot.getSLIDES_PUT_SP_ON_HIGH_RUNG());//slides down to clip specimen
        robot.updateSlidesPos();
        robot.delay(3);

        robot.setGrasperPos(robot.getGRASPER_OPEN());//release specimen
        robot.updateGrasperPos();
        robot.delay(.3);

        robot.setSlidesPos(robot.getSLIDES_ABOVE_HIGH_RUNG());//slides back up
        robot.updateSlidesPos();
        robot.delay(.5);

        Action moveBackAwayFromSub = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-12, 43))
                .build();

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

        Action moveToFirstRedSample = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-36.5, 36))
                .strafeToLinearHeading(new Vector2d(-47, 12), Math.toRadians(-180))
                .strafeToLinearHeading(new Vector2d(-49, 12), Math.toRadians(-270))
                .build();

        Actions.runBlocking(moveToFirstRedSample);//drive to first red sample on ground
        robot.delay(.2);


        Action moveFirstSampleToOZ = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-49, 56.5))
                .build();

        Actions.runBlocking(moveFirstSampleToOZ);//push first red sample to observation zone
        robot.delay(.2);

        Action moveToSecondSample = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-49, 12))
                .strafeToConstantHeading(new Vector2d(-58.5, 12))
                .build();
        Actions.runBlocking(moveToSecondSample);//move to second red sample
        robot.delay(.2);


        Action moveSecondSampleToOZ = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-58.5, 56.5))
                .build();

        Actions.runBlocking(moveSecondSampleToOZ);
        robot.delay(.2);


        Action turnAround = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-55, 54))
                .strafeToLinearHeading(new Vector2d(-55, 54), Math.toRadians(-220))
                .build();
        Actions.runBlocking(turnAround);


        robot.resetAll();
        //servos
        robot.updateAxleServoPos();
        robot.updateGimbalPos();
        robot.updateGrasperPos();
        //motors
        robot.updatePivotMotorPos();
        robot.updateSlidesPos();

        robot.delay(5);

        telemetry.addData("slides pos", robot.getLeftSliderPos());
        telemetry.addLine("this auto puts specimen on high chamber, and then moves to push 2 red samples (one at a time) to the Observation zone.");
        telemetry.update();

    }
}


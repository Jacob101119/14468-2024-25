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

                .strafeToConstantHeading(new Vector2d(11.4, -43.5))
                .build();













        waitForStart();
//new updates to run movements


        Actions.runBlocking(moveToSub);

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

        //slides up

//don't use anymore, moves forward farther but not necessary
        Action moveSlidesOverHighRung = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(11.40, -36), Math.toRadians(90))
                .build();


        //Actions.runBlocking(moveSlidesOverHighRung);
        //robot.delay(.5);
        //----
        robot.setSlidesPos(robot.getSLIDES_PUT_SP_ON_HIGH_RUNG()-100);//clip specimen
        robot.updateSlidesPos();
        robot.delay(1);

        Action moveAnInch = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(11.4, -39.5))
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

        //robot.setSlidesPos(robot.getSLIDES_ABOVE_HIGH_RUNG());//slides back up
        //robot.updateSlidesPos();
        //robot.delay();



        Action moveBackAwayFromSub = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(12.00, -41.00))
                .build();

        Actions.runBlocking(moveBackAwayFromSub);

        robot.setSlidesPos(0);//slides down
        robot.updateSlidesPos();
        robot.delay(.7);


        Action moveToFirstRedSample = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(36.5, -36))
                .strafeToLinearHeading(new Vector2d(47, -12), 0)
                .strafeToLinearHeading(new Vector2d(49, -12), -90)
                .build();
        Actions.runBlocking(moveToFirstRedSample);//drive to first red sample on ground
        robot.delay(.2);


        Action moveFirstSampleToOZ = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(49, -56.5))
                .build();

        Actions.runBlocking(moveFirstSampleToOZ);//push first red sample to observation zone
        robot.delay(.2);


        Action moveToSecondSample = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(49, -12))
                .strafeToConstantHeading(new Vector2d(58.5, -12))
                .build();
        Actions.runBlocking(moveToSecondSample);//move to second red sample
        robot.delay(.2);


        Action moveSecondSampleToOZ = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(58.5, -56.5))
                .build();

        Actions.runBlocking(moveSecondSampleToOZ);
        robot.delay(.2);


        Action turnAround = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(55, -54))
                .strafeToLinearHeading(new Vector2d(55, -54), 130)
                .build();

        //Actions.runBlocking();



    }
}


package org.firstinspires.ftc.teamcode.autoMeet1;


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

    public void runOpMode() throws InterruptedException {
        robot = new BaseRobot(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();
//new updates to run movements
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)


                        .strafeToLinearHeading(new Vector2d(23.75, 1), 0)//to sub

                        .strafeToLinearHeading(new Vector2d(23.75, -24), 0)//strafe right

                        .strafeToLinearHeading(new Vector2d(48.75, -37), 20)//strafe step 1 to sample 1

                        .strafeToLinearHeading(new Vector2d(48.75, -26), 20)//sample 1 step 2

                        .strafeToLinearHeading(new Vector2d(48.75, -37), 180)//aligned with sample 1

                        .strafeToLinearHeading(new Vector2d(6.75, -37), 180)//1st sample in observation zone

                        .strafeToLinearHeading(new Vector2d(51.75, -36), 180)//move back to go for second sample

                        .strafeToLinearHeading(new Vector2d(49.75, -46.5), 180)//aligned with sample 2

                        .strafeToLinearHeading(new Vector2d(6.75, 46.5), 180)//put second sample in observation zone


                        .build());








    }
}


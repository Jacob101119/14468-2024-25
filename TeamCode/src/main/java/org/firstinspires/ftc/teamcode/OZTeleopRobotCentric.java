package org.firstinspires.ftc.teamcode;

//import com.google.blocks.ftcrobotcontroller.runtime.CRServoAccess;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;

@TeleOp
public class OZTeleopRobotCentric extends LinearOpMode {

    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    BaseRobot robot;



    @Override
    public void runOpMode() throws InterruptedException {




        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        robot = new BaseRobot(hardwareMap);

        while(!isStopRequested() && opModeIsActive()){


            robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x));


            //updates
            robot.updateGimbalPos();
            robot.updateHangArmPos();
            robot.updatePivotMotorPos();
            robot.updateSlidesPos();
            robot.updateGrasperPos();
            robot.updateAxleServoPos();
            //end




            //slides
            //robot.changeSlidesPos((int)(-gamepad2.right_stick_y * 10));
            robot.changeSlidesPos((int)(gamepad2.right_trigger - gamepad2.left_trigger) * 10);
            robot.changeSlidesPos((int)(-gamepad2.left_stick_y) * 10);
            //slides position changes from inputs gamepad1 triggers and gamepad 2 right stick


            //pivot motor
            /*if (gamepad1.right_bumper){
                robot.changePivotMotorPos(10);
            }
            if (gamepad1.left_bumper){
                robot.changePivotMotorPos(-10);
            }

             */

            robot.changePivotMotorPos((int) (gamepad2.right_stick_y * 12));
            //robot.changeHangArmPos((int) ((gamepad2.right_trigger-gamepad2.left_trigger) * 10));


            /*if(gamepad1.dpad_right){
                robot.setGrasperPos(robot.GRASPER_WIDE_OPEN);
            }

             */

            if(gamepad2.right_bumper){
                robot.resetSliderEncoders();
            }

            if(gamepad2.dpad_up){
                robot.setPivotMotorPos(robot.getPIVOT_MOTOR_HORIZONTAL()-1000);
            }
            if (gamepad1.x){
                robot.setGrasperPos(robot.getGRASPER_OPEN());
            }
            if(gamepad1.a){
                robot.setGrasperPos(robot.GRASPER_CLOSED);
            }

            if(gamepad1.dpad_up || gamepad2.dpad_up){
                robot.setAxlePos(robot.getAXLE_SERVO_UP());
            }
            if(gamepad2.dpad_down){
                robot.setAxlePos(robot.getAXLE_SERVO_BACK());
            }
            if (gamepad1.dpad_right){
                robot.setAxlePos(robot.getAXLE_SERVO_BACK());
            }

            //if (gamepad1.dpad_up){
              //  robot.setPivotMotorPos(robot.getPIVOT_MOTOR_VERTICAL());
            //}
            if(gamepad1.left_bumper){
                robot.changeGimbalPos(.01);
            }
            if(gamepad1.right_bumper){
                robot.changeGimbalPos(-.01);
            }
            if(gamepad1.left_bumper && gamepad2.right_bumper){
                robot.setGimbalPos(robot.GIMBAL_RESTING_POS);
            }

            if (gamepad2.a){
                //grab specimen from wall
                robot.setAxlePos(robot.getAXLE_SERVO_GRAB_FROM_WALL());
                robot.setGimbalPos(robot.getGIMBAL_RESTING_POS());
                robot.setSlidesPos(0);//slides down
                robot.setPivotMotorPos(robot.getPIVOT_MOTOR_GRAB_FROM_WALL()-180);

            }
            if (gamepad2.x){
                robot.setPivotMotorPos(3800);


            }
            if (gamepad2.y){
                robot.setPivotMotorPos(robot.PIVOT_MOTOR_VERTICAL+100);
                robot.setSlidesPos(robot.getSLIDES_ABOVE_HIGH_RUNG());
                //robot.setAxlePos(robot.getAXLE_SERVO_UP());
                //robot.setAxlePos(robot.getAXLE_SERVO_UP());
            }
            if (gamepad2.b && !gamepad1.start && !gamepad2.start){
                robot.setPivotMotorPos(2248);
                robot.setSlidesPos(1850);
            }
            if(gamepad2.dpad_right){
                robot.setPivotMotorPos(robot.PIVOT_MOTOR_HORIZONTAL);
                robot.setAxlePos(robot.getAXLE_SERVO_UP());
                robot.setGimbalPos(robot.getGIMBAL_RESTING_POS());
            }
            if(gamepad1.right_stick_button){
                robot.setGimbalPos(robot.getGIMBAL_RESTING_POS());
            }







            //telemetry
            //_____________________________________________________________________________________



           // telemetry.addLine("robot position (starting at x: 0, y: 0, heading: 0)");
            //telemetry.addData("x:", drive.pose.position.x);
            //telemetry.addData("y:", drive.pose.position.y);



            telemetry.addData("Position", robot.drive.pose);
            telemetry.addData("x: ", robot.drive.pose.position.x);
            telemetry.addData("y: ", robot.drive.pose.position.y);
            telemetry.addData("Heading: ", Math.toDegrees(robot.drive.pose.heading.toDouble()));





            telemetry.addLine();
            telemetry.addLine();


            telemetry.addLine("Motors: ");
            telemetry.addLine();

            telemetry.addLine("Slides: ");
            telemetry.addData("left slides position: ", robot.getLeftSliderPos());
            telemetry.addData("left slide power: ", robot.getLEFT_SLIDE_POWER());
            telemetry.addLine();

            telemetry.addData("right slides position: ", robot.getRightSliderPos());
            telemetry.addData("right slides power: ", robot.getRIGHT_SLIDE_POWER());
            telemetry.addLine();
            telemetry.addLine();

            //telemetry.addData("hang arm position: ", robot.getHangArmPos());

            telemetry.addLine("Pivot motor: ");
            telemetry.addData("pivot motor position: ", robot.getPivotMotorPos());
            telemetry.addData("pivot motor power: ", robot.getPIVOT_MOTOR_POWER());

            telemetry.addLine();
            telemetry.addLine("Servos: ");
            telemetry.addData("gimbal servo pos" , robot.getGimbalPos());
            telemetry.addData("Axle servo pos" , robot.getAxlePos());
            telemetry.addData("grasper pos" , robot.getGrasperPos());

            telemetry.addLine();
            telemetry.addLine();

            telemetry.addLine("2-a: grab from wall");
            telemetry.addLine("2-y: specimen scoring");
            telemetry.addLine("2-x: grab from ground");

            /*
            telemetry.addLine("controls: ");
            telemetry.addLine();

            telemetry.addLine("Gamepad1:");
            telemetry.addLine("right/left stick: drive");
            //telemetry.addLine("right/left trigger: slides");
            telemetry.addLine();

            telemetry.addLine("Gamepad2:");
            telemetry.addLine("left stick: pivot motor");
            telemetry.addLine("right stick: slides");
            telemetry.addLine("dpad_up/down: axle rotation servo");
            telemetry.addLine("dpad_left/right: grasper");
            telemetry.addLine("a: pivot motor back");
            telemetry.addLine("y: pivot motor vertical");
            telemetry.addLine("b: pivot motor horizontal");
            telemetry.addLine("left/right bumper: change gimbal pos");
            telemetry.addLine("left&right bumper at the same time: reset gimbal pos");
             */
            telemetry.update();
            //end telemetry



            //_____________________________________________________________________________________
        }
    }
}

package org.firstinspires.ftc.teamcode;

//import com.google.blocks.ftcrobotcontroller.runtime.CRServoAccess;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class GraysonTeleop extends LinearOpMode {

    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    BaseRobot robot;



    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive d = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        robot = new BaseRobot(hardwareMap);



        waitForStart();

        while(!isStopRequested() && opModeIsActive()){


            robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x));


            //updates
            robot.updateGimbalPos();
            robot.updateHangArmPos();
            robot.updatePivotMotorPos();
            robot.updateSlidesPos();
            //end


            //gimbal servo

            //end gimbal servo


            //slides
            robot.changeSlidesPos((int)(-gamepad2.right_stick_y * 10));
            robot.changePivotMotorPos((int) (gamepad2.left_stick_y * 10));
            robot.changeHangArmPos((int) ((gamepad2.right_trigger-gamepad2.left_trigger) * 10));




            if(gamepad1.a){
                robot.setGrasperPos(robot.GRASPER_WIDE_OPEN);
            }
            if(gamepad1.x){
                robot.setGrasperPos(robot.GRASPER_CLOSED);
            }

            if(gamepad2.a){
                robot.setAxlePos(.2);
            }
            if(gamepad2.b){
                robot.setAxlePos(0);
            }
            if(gamepad2.y){
                robot.setAxlePos(.7);
            }
            if(gamepad2.x){
                robot.setAxlePos(.8);
            }

            if(gamepad2.left_bumper){
                robot.changeGimbalPos(-.05);
            }
            if(gamepad2.right_bumper){
                robot.changeGimbalPos(.05);
            }
            if(gamepad1.right_stick_button){
                robot.setGimbalPos(robot.GIMBAL_RESTING_POS);
            }



            //end slides
            //_____________________________________________________________________________________

            /*
            if (gamepad1.dpad_left){
                robot.reachToSub();
            }
            if (gamepad1.dpad_down){
                robot.resetAll();
            }
            if(gamepad1.dpad_up){
                robot.basketScoring();

            }
            */


            //if (gamepad1.dpad_up){
            //  robot.basketScoring();
            //}
            //if (gamepad1.y){
            //  robot.specimenScoring();
            //}


            //telemetry
            //_____________________________________________________________________________________

            telemetry.addData("left slides position: ", robot.getLeftSliderPos());
            telemetry.addData("right slides position: ", robot.getRightSliderPos());
            telemetry.addData("hang arm position: ", robot.getHangArmPos());
            telemetry.addData("right slides position: ", robot.getRightSliderPos());
            telemetry.addData("pivot motor position: ", robot.getPivotMotorPos());
            telemetry.addData("gimbal servo pos" , robot.getGimbalPos());


            telemetry.update();
            //end telemetry
            //_____________________________________________________________________________________
        }
    }
}

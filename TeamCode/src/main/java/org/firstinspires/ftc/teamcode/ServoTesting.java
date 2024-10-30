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
public class ServoTesting extends LinearOpMode {

    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    BaseRobot robot;



    @Override
    public void runOpMode() throws InterruptedException {


        robot = new BaseRobot(hardwareMap);



        waitForStart();

        while(!isStopRequested() && opModeIsActive()){

            //robot.setAxlePos(robot.getAXLE_SERVO_UP());


            robot.updateAxleServoPos();
            robot.updateGrasperPos();
            robot.updateGimbalPos();

            robot.changeAxlePos(gamepad1.right_stick_y * 0.01);
            robot.changeGrasperPos(gamepad1.left_stick_y * 0.01);

            if(gamepad1.y){
                robot.changeGrasperPos(.001);
            }
            if(gamepad1.a){
                robot.changeGrasperPos(-.001);
            }
            if(gamepad1.x){
                robot.changeAxlePos(.001);
            }
            if(gamepad1.b){
                robot.changeAxlePos(-.001);
            }
            if(gamepad1.dpad_up){
                robot.changeGimbalPos(.001);
            }
            if(gamepad1.dpad_down){
                robot.changeGimbalPos(-.001);
            }

            if(gamepad2.a){
                robot.setAxlePos(robot.getAXLE_SERVO_UP());
            }

            //telemetry
            //_____________________________________________________________________________________

            telemetry.addLine("axle servo: right stick & x/b");
            telemetry.addData("axle servo pos", robot.getAxlePos());
            telemetry.addLine();

            telemetry.addLine("gimbal servo: dpad_up and dpad_down");
            telemetry.addData("gimbal servo pos", robot.getGimbalPos());
            telemetry.addLine();

            telemetry.addLine("grasper: left stick & a/y");
            telemetry.addData("grasper servo pos", robot.getGrasperPos());
            telemetry.addLine();

            telemetry.update();
            //end telemetry
            //_____________________________________________________________________________________
        }
    }
}

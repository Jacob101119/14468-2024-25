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
public class ResetTeleop extends LinearOpMode {

    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    BaseRobot robot;



    @Override
    public void runOpMode() throws InterruptedException {


        robot = new BaseRobot(hardwareMap);



        waitForStart();

        while(!isStopRequested() && opModeIsActive()){



            //updates
            robot.RESETUpdatePivotMotorPos();
            robot.updateSlidesPos();
            //end

            robot.setGimbalPos(robot.getGIMBAL_RESTING_POS());
            robot.setAxlePos(robot.getAXLE_SERVO_DOWN());
            robot.setGrasperPos(robot.getGRASPER_OPEN());
            robot.updateAxleServoPos();
            robot.updateGimbalPos();
            robot.updateGrasperPos();



            //slides
            robot.changeSlidesPos((int)(-gamepad1.right_stick_y * 1));
            robot.changePivotMotorPos((int) (-gamepad1.left_stick_y * 2));

            robot.changeSlidesPos((int)(-gamepad2.right_stick_y * 5));
            robot.changePivotMotorPos((int) (-gamepad2.left_stick_y * 5));


            if (gamepad1.left_bumper){
                robot.changeGimbalPos(-.05);
            }
            if (gamepad1.right_bumper){
                robot.changeGimbalPos(.05);
            }


            //end slides
            //_____________________________________________________________________________________





            //telemetry
            //_____________________________________________________________________________________
            telemetry.addLine("Controls: (gamepad1 for slower movements, 2 for faster):");
            telemetry.addLine("Slides: gamepad1 - right stick");
            telemetry.addLine("pivot motor: gamepad1 - left stick");
            telemetry.addLine("gimbal servo: left/right bumper");
            telemetry.addLine("----------------------------------");
            telemetry.addLine("Positions:");
            telemetry.addData("left slides position: ", robot.getLeftSliderPos());
            telemetry.addData("right slides position: ", robot.getRightSliderPos());

            telemetry.addData("right slides position: ", robot.getRightSliderPos());
            telemetry.addData("pivot motor position: ", robot.getPivotMotorPos());
            telemetry.addData("gimbal servo position:", robot.getGimbalPos());


            telemetry.update();
            //end telemetry
            //_____________________________________________________________________________________
        }
    }
}

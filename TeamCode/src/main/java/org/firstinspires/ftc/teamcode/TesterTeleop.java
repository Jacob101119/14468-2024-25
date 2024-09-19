package org.firstinspires.ftc.teamcode;

//import com.google.blocks.ftcrobotcontroller.runtime.CRServoAccess;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TesterTeleop extends LinearOpMode {

    BaseRobot robot;



    @Override
    public void runOpMode() throws InterruptedException {

        robot = new BaseRobot(hardwareMap);


        waitForStart();

        while(!isStopRequested() && opModeIsActive()){

            //worm gear box
            robot.pivotMotor.setPower(-gamepad1.left_stick_y);
            //end worm gear box

            //hang arm
            if(gamepad1.y){
                robot.hangArm.setPower(.2);
            }
            else if (gamepad1.a){
                robot.hangArm.setPower(-.2);
            }
            //end hang arm

            //slides
            double TestSlidesPower = -gamepad1.right_stick_y;
            robot.leftSlider.setPower(TestSlidesPower);
            robot.rightSlider.setPower(TestSlidesPower);

            //if (gamepad1.dpad_down){
              //  slidesDown();
            //}
            //end slides


            //servo test

            //end servo test


            //telemetry.addData("left stick: slides pivot motor");

        }
    }
}

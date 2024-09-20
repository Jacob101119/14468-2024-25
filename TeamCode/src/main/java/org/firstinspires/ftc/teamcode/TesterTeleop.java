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
public class TesterTeleop extends LinearOpMode {

    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    BaseRobot robot;

    int leftSliderPos = 0;
    int rightSliderPos = 0;

    int hangArmPos = 0;

    int pivotMotorPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive d = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        robot = new BaseRobot(hardwareMap);

        //motor encoder setup
        robot.hangArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hangArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangArmPos = robot.hangArm.getCurrentPosition();

        robot.leftSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSliderPos = robot.leftSlider.getCurrentPosition();
        rightSliderPos = robot.rightSlider.getCurrentPosition();

        robot.pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotorPos = robot.pivotMotor.getCurrentPosition();
        //end

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){


        robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(gamepad2.left_stick_x, gamepad2.left_stick_y), gamepad2.right_stick_y));

            //worm gear box
            robot.pivotMotor.setPower(-gamepad1.left_stick_y);
            //end worm gear box

            //hang arm
            double hangArmMotorPower = gamepad1.right_trigger - gamepad1.left_trigger;
            //int hangArmMotorDelta = (int) ((gamepad1.right_trigger - gamepad1.left_trigger) * 10);
            if(hangArmMotorPower > .1){
                hangArmPos += gamepad1.right_trigger * 10;//how would i get this to work with trigger
            }
            if(hangArmMotorPower < -.1){
                hangArmPos -= gamepad1.right_trigger * 10;
            }
            //end hang arm
            //_____________________________________________________________________________________

            //slides
            int  slidesMotorDelta = (int) (gamepad1.right_stick_y * 10);

                if(Math.abs(slidesMotorDelta) > .1) {
                leftSliderPos += slidesMotorDelta;
                rightSliderPos += slidesMotorDelta;
            }
            if (gamepad1.dpad_down){
                robot.slidesDown();//sets slides pos to 0
            }



            robot.leftSlider.setTargetPosition(leftSliderPos);
            robot.rightSlider.setTargetPosition(rightSliderPos);

            robot.leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftSlider.setPower(1);
            robot.rightSlider.setPower(1);

            if(rightSliderPos < 0){
                rightSliderPos = 0;
            }
            if(leftSliderPos < 0){
                leftSliderPos = 0;
            }
            if(leftSliderPos > 3000) {//change number
                leftSliderPos = 3000;
            }
            if(rightSliderPos > 3000) {//change number
                rightSliderPos = 3000;
            }


            //end slides
            //_____________________________________________________________________________________


            if (gamepad1.dpad_left){
                robot.reachToSub();
            }
            if (gamepad1.dpad_right){
                robot.sliderReset();
            }
            if (gamepad1.dpad_up){
                robot.highScoring();
            }


            //telemetry
            //_____________________________________________________________________________________
            telemetry.addData("left slides position: ", leftSliderPos);
            telemetry.addData("hang arm position: ", hangArmPos);
            telemetry.addData("right slides position: ", rightSliderPos);
            telemetry.addData("pivot motor position: ", pivotMotorPos);
            telemetry.addData("Slides motor delta: ", slidesMotorDelta);

            telemetry.update();
            //end telemetry
            //_____________________________________________________________________________________
        }
    }
}

package org.firstinspires.ftc.teamcode;

//import com.google.blocks.ftcrobotcontroller.runtime.CRServoAccess;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TesterTeleop extends LinearOpMode {

    BaseRobot robot;

    int leftSliderPos = 0;
    int rightSliderPos = 0;

    int hangArmPos = 0;

    int pivotMotorPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {



        robot = new BaseRobot(hardwareMap);

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

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){



            //worm gear box
            robot.pivotMotor.setPower(-gamepad1.left_stick_y);
            //end worm gear box

            //hang arm
            double hangArmMotorPower = gamepad1.right_trigger - gamepad1.left_trigger;
            //int hangArmMotorDelta = (int) ()
            if(hangArmMotorPower > .1){
                hangArmPos += gamepad1.right_trigger * 10;
            }
            if(hangArmMotorPower < -.1){
                hangArmPos -= gamepad1.right_trigger * 10;
            }
            //end hang arm

            int  slidesMotorDelta = (int) (gamepad1.right_stick_y * 10);

            if(slidesMotorDelta > 1){
                leftSliderPos += slidesMotorDelta;
                rightSliderPos += slidesMotorDelta;
            }

            if (slidesMotorDelta < -1){
                leftSliderPos -= slidesMotorDelta;
                rightSliderPos -= slidesMotorDelta;
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
                //leftSliderPos = 3000 change number
            }
            if(rightSliderPos > 3000) {//change number
                //leftSliderPos = 3000 change number
            }


            if (gamepad1.dpad_down){
                //slidesDown(); //sets slides position to 0
            }



            telemetry.addData("left slides position: ", leftSliderPos);
            telemetry.addData("hang arm position: ", hangArmPos);
            telemetry.addData("right slides position: ", rightSliderPos);
            telemetry.addData("pivot motor position: ", pivotMotorPos);
            telemetry.addData("Slides motor delta: ", slidesMotorDelta);

            telemetry.update();
        }
    }
}

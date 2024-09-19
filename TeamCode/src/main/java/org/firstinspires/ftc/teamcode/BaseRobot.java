package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BaseRobot {

    DcMotor leftSlider;//from the perspective of the robot
    DcMotor rightSlider;//from the perspective of the robot
    DcMotor pivotMotor;//worm gear box
    DcMotor hangArm;//monkey arm




    public BaseRobot(HardwareMap hwMap){

        hangArm = hwMap.dcMotor.get("hangArm");
        //hangArm.setPower(1);

        pivotMotor = hwMap.dcMotor.get("pivotMotor");
        //pivotMotor.setPower(1);

        leftSlider = hwMap.dcMotor.get("leftSlider");
        rightSlider = hwMap.dcMotor.get("rightSlider");
        rightSlider.setDirection(DcMotorSimple.Direction.REVERSE);

        //Servo claw  = null;
        //claw = hwMap.servo.get("claw");
        //leftHand.setPosition(MID_SERVO);?????????









    }

    public void sliderReset() {
        //make slider position 0
        leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlider.setTargetPosition(0);
        rightSlider.setTargetPosition(0);
    }
    public void slidesUp(){
        int slidesMax = 10;//change
        leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlider.setTargetPosition(slidesMax);
        leftSlider.setTargetPosition(slidesMax);
    }
    public void slidesDown(){
        leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlider.setTargetPosition(0);
        leftSlider.setTargetPosition(0);
    }
    public void sub() {

        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setTargetPosition(0);//change
        //set worm gear down and slides up
    }
    public void sliderRunTo(int position){
        //leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //leftSlider.setTargetPosition(position);
        //rightSlider.setTargetPosition(position);
        //leftSlider.setPower(1);
        //rightSlider.setPower(1);



    }
}

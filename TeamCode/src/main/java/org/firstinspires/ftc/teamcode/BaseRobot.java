package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class BaseRobot{

    public MecanumDrive drive;

    DcMotor leftSlider;//from the perspective of the robot
    DcMotor rightSlider;//from the perspective of the robot
    DcMotor pivotMotor;//worm gear box
    DcMotor hangArm;//monkey arm



    int slidesMax = 5000;//change
    int pivotMotorVertical = 0; //change
    int pivotMotorHorizontal = 0; //change


    public BaseRobot(HardwareMap hwMap){

        drive = new MecanumDrive(hwMap, new Pose2d(0, 0, 0));

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
        pivotMotor.setTargetPosition(pivotMotorVertical);
    }
    public void slidesUp(){
        int slidesMax = 10;//change
        leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlider.setTargetPosition(slidesMax);
        leftSlider.setTargetPosition(slidesMax);
    }
    public void highScoring(){
        pivotMotor.setTargetPosition(pivotMotorVertical);
        leftSlider.setTargetPosition(slidesMax);
        rightSlider.setTargetPosition(slidesMax);

    }
    public void slidesDown(){
        leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlider.setTargetPosition(0);
        leftSlider.setTargetPosition(0);
    }
    public void reachToSub() {
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //pivotMotor.setTargetPosition();
        //rightSlider.setTargetPosition();//half ish pos
        //leftSlider.setTargetPosition();//half ish pos
    }
    public void sliderRunTo(int position){
        //leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //leftSlider.setTargetPosition(position);
        //rightSlider.setTargetPosition(position);
        //leftSlider.setPower(1);
        //rightSlider.setPower(1);
    }

    public void updateArmPos() {
        // Code from teleop that grabs armPos variables and
        // Uses run to position
    }

    public void setArmPos(int newPos) {
        // update the fields (variables) that hold armpos
        // Like:  leftArmPos = newPos;
    }

    public void changeArmPos(int deltaPos) {
        // update the fields (variables) by adding deltaPos
        // Like: leftArmPos += deltaPos
    }
}

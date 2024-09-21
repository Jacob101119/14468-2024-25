package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class BaseRobot{

    // servo Constants
    double grasperOpen = 0.5;//change
    double grasperClosed = 0;//change
    double axleServoOut = 0; //change
    double axleServoUp = 0;//change
    double axleServoMid = 0; //change
    double gimbalBasketScoring = 0;//change
    double gimbalSpecimenScoring = 0;//change
    double gimbalRestingPos = 0;//change
    double axleServoDown = 0;//change
    //end servo constants

    //motor constants
    int slidesMax = 3000;//change
    int pivotMotorVertical = 0; //change
    int pivotMotorHorizontal = 0; //change
    int slidesMin = 0;
    //end motor constants

    public MecanumDrive drive;

    DcMotor leftSlider;//from the perspective of the robot
    DcMotor rightSlider;//from the perspective of the robot
    DcMotor pivotMotor;//worm gear box
    DcMotor hangArm;//monkey arm

    //servos
    Servo grasper  = null;
    Servo grasperGimbal = null;
    Servo axleRotation = null;

    //end servos




    public BaseRobot(HardwareMap hwMap){

        drive = new MecanumDrive(hwMap, new Pose2d(0, 0, 0));

        hangArm = hwMap.dcMotor.get("hangArm");
        //hangArm.setPower(1);

        pivotMotor = hwMap.dcMotor.get("pivotMotor");
        //pivotMotor.setPower(1);

        leftSlider = hwMap.dcMotor.get("leftSlider");
        rightSlider = hwMap.dcMotor.get("rightSlider");
        rightSlider.setDirection(DcMotorSimple.Direction.REVERSE);

        //servos
        grasper = hwMap.servo.get("claw");
        grasper.setPosition(grasperOpen);

        grasperGimbal = hwMap.servo.get("grasperGimbal");
        grasperGimbal.setPosition(0);

        axleRotation = hwMap.servo.get("axleRotation");
        axleRotation.setPosition(0);
        //end servos









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
    public void basketScoring(){
        pivotMotor.setTargetPosition(pivotMotorVertical);
        leftSlider.setTargetPosition(slidesMax);
        rightSlider.setTargetPosition(slidesMax);
        axleRotation.setPosition(axleServoUp);
        grasperGimbal.setPosition(gimbalBasketScoring);

    }
    public void specimenScoring(){
        pivotMotor.setTargetPosition(pivotMotorVertical);
        leftSlider.setTargetPosition(slidesMax);//maybe a bit less
        rightSlider.setTargetPosition(slidesMax);//maybe a bit less
        axleRotation.setPosition(axleServoMid);
        grasperGimbal.setPosition(gimbalSpecimenScoring);
    }
    public void slidesDown(){
        leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlider.setTargetPosition(0);
        leftSlider.setTargetPosition(0);
    }
    public void reachToSub() {
        //grasper.setPosition(grasperClosed);
        //pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //pivotMotor.setTargetPosition();
        //rightSlider.setTargetPosition();//half ish pos
        //leftSlider.setTargetPosition();//half ish pos
        //axleRotation.setPosition(axleServoOut);
    }
    public void sliderRunTo(int position){
        leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlider.setTargetPosition(position);
        rightSlider.setTargetPosition(position);
        leftSlider.setPower(1);
        rightSlider.setPower(1);
    }
    public void pivotRunTo(int position){
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setTargetPosition(position);
        pivotMotor.setPower(1);
    }
    public void hangArmRunTo(int position){
        hangArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangArm.setTargetPosition(position);
        hangArm.setPower(1);
    }
    public void SlidesReset(){
        grasper.setPosition(grasperClosed);
        leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlider.setTargetPosition(0);
        leftSlider.setTargetPosition(0);
        grasperGimbal.setPosition(gimbalRestingPos);
        axleRotation.setPosition(axleServoDown);


    }


    /*public void updateSlidesPos() {

        // Code from teleop that grabs armPos variables and
        // Uses run to position
    }

    public void setSlidesPos(int newPos) {
        //leftSliderPos = newPos;
        //rightSliderPos = newPos;
        // update the fields (variables) that hold armpos

    }

    public void changeSlidesPos(int deltaPos) {
        //leftSliderPos += deltaPos;
        //rightSliderPos += deltaPos;
        // update the fields (variables) by adding deltaPos

    }

     */
}

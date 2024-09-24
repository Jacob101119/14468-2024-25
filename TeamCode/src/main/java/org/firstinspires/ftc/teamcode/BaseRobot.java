package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class BaseRobot{

    // Arm Position fields
    private int leftSliderPos = 0;
    private int rightSliderPos = 0;
    private int pivotMotorPos = 0;
    private int gimbalPos = 0;
    private int hangArmPos = 0;

    private int deltaLeftPos = 0;
    private int deltaRightPos = 0;

    private double PIVOT_MOTOR_POWER = .8;
    private double LEFT_SLIDE_POWER = 0.8;
    private double RIGHT_SLIDE_POWER = 0.8;
    private double HANG_ARM_POWER = 0.8;

    // servo Constants
    double GRASPER_OPEN = 0.5;//change
    double GRASPER_CLOSED = 0;//change
    double AXLE_SERVO_BACK = 0; //change
    double AXLE_SERVO_UP = 0;//correct
    double AXLE_SERVO_DOWN = .8;//correct
    double AXLE_SERVO_OUT = .4; //correct
    double GIMBAL_BASKET_SCORING = 0;//change
    double GIMBAL_SPECIMEN_SCORING = 0;//change
    double GIMBAL_RESTING_POS = 0;//change


    //end servo constants

    //motor constants
    int SLIDES_MAX = 3000;//change
    int SLIDES_TO_SUB = 60;//change
    int PIVOT_MOTOR_TO_SUB = 0;//change
    int PIVOT_MOTOR_VERTICAL = 0; //change
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

        // Set to run with encoders and grab current Position
        leftSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSliderPos = leftSlider.getCurrentPosition();

        rightSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSliderPos = rightSlider.getCurrentPosition();

        //hang arm
        hangArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangArmPos = hangArm.getCurrentPosition();

        //pivot motor
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotorPos= hangArm.getCurrentPosition();

        //servos
        grasper = hwMap.servo.get("claw");
        grasper.setPosition(GRASPER_OPEN);

        grasperGimbal = hwMap.servo.get("grasperGimbal");
        grasperGimbal.setPosition(0);

        axleRotation = hwMap.servo.get("axleRotation");
        axleRotation.setPosition(0);
        //end servos



    }


    public void slidesUp(){
        int slidesMax = 10;//change
        leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlider.setTargetPosition(slidesMax);
        leftSlider.setTargetPosition(slidesMax);
    }
    public void basketScoring(){
        pivotMotor.setTargetPosition(PIVOT_MOTOR_VERTICAL);
        leftSlider.setTargetPosition(SLIDES_MAX);
        rightSlider.setTargetPosition(SLIDES_MAX);
        axleRotation.setPosition(AXLE_SERVO_UP);
        grasperGimbal.setPosition(GIMBAL_BASKET_SCORING);

    }
    public void specimenScoring(){
        pivotMotor.setTargetPosition(PIVOT_MOTOR_VERTICAL);
        leftSlider.setTargetPosition(SLIDES_MAX);//maybe a bit less
        rightSlider.setTargetPosition(SLIDES_MAX);//maybe a bit less
        axleRotation.setPosition(AXLE_SERVO_OUT);
        grasperGimbal.setPosition(GIMBAL_SPECIMEN_SCORING);
    }
    public void slidesDown(){
        setSlidesPos(slidesMin);
    }
    public void reachToSub() {

        axleRotation.setPosition(AXLE_SERVO_OUT);
        setSlidesPos(SLIDES_TO_SUB);//change number
        setPivotMotorPos(PIVOT_MOTOR_TO_SUB);//change
        grasper.setPosition(GRASPER_OPEN);
    }



    //hang arm

    public void slidesReset(){
        grasper.setPosition(GRASPER_CLOSED);
        setSlidesPos(slidesMin);
        grasperGimbal.setPosition(GIMBAL_RESTING_POS);
        axleRotation.setPosition(AXLE_SERVO_DOWN);
    }
    public void slidesMax(){
        setSlidesPos(SLIDES_MAX);
    }
    //end hang arm

    //hang arm pos
    public void updateHangArmPos(){
        hangArm.setTargetPosition(hangArmPos);
        hangArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangArm.setPower(HANG_ARM_POWER);
    }
    public void setHangArmPos(int newPos) {
        hangArmPos = newPos;
    }
    public void changeHangArmPos(int deltaPos){
        hangArmPos += deltaPos;
    }
    //end hang arm


    //pivot motor pos
    public void updatePivotMotorPos() {
        pivotMotor.setTargetPosition(pivotMotorPos);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setPower(PIVOT_MOTOR_POWER);
    }
    public void setPivotMotorPos(int newPos) {
        pivotMotorPos = newPos;
    }
    public void changePivotMotorPos(int deltaPos){
        pivotMotorPos += deltaPos;
    }




    public void updateGimbalPos() {
        grasperGimbal.setPosition(gimbalPos);
    }
    public void setGimbalPos(int newPos){
        gimbalPos = newPos;

    }
    public void changeGimbalPos(double deltaPos){
        gimbalPos += deltaPos;
    }

    public void updateSlidesPos() {

        // Code from teleop that grabs armPos variables and
        // Uses run to position
        if(rightSliderPos > 3000){
            rightSliderPos = 3000;
        }
        if (leftSliderPos > 3000){
            leftSliderPos = 3000;
        }
        if(rightSliderPos < 0){
            rightSliderPos = 0;
        }
        if (leftSliderPos < 0){
            leftSliderPos = 0;
        }

        rightSlider.setTargetPosition(rightSliderPos);
        rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlider.setTargetPosition(leftSliderPos);
        leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlider.setPower(LEFT_SLIDE_POWER);
        rightSlider.setPower(RIGHT_SLIDE_POWER);

    }
    public void setSlidesPos(int newPos) {

        leftSliderPos = newPos;
        rightSliderPos = newPos;
        //rightSliderPos = newPos;
        // update the fields (variables) that hold armpos

    }

    public void changeSlidesPos(int deltaPos) {
        leftSliderPos += deltaPos;
        rightSliderPos += deltaPos;

        // update the fields (variables) by adding deltaPos

    }


}

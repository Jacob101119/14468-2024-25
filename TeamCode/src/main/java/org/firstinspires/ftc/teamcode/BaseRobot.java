package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class BaseRobot{

    // Arm Position fields
    private int leftSliderPos = 0;
    private int rightSliderPos = 0;
    private int pivotMotorPos = 0;
    private int hangArmPos = 0;
    //end arm pos fields

    //servo positions
    private double gimbalPos = 0;
    private double axlePos = 0;
    private double grasperPos = 0;
    //end servo positions





    //private int deltaLeftPos = 0;
    //private int deltaRightPos = 0;

    //motor powers
    private double PIVOT_MOTOR_POWER = 0.4;
    private double LEFT_SLIDE_POWER = 0.8;
    private double RIGHT_SLIDE_POWER = 0.8;
    private double HANG_ARM_POWER = 0.8;
    //end

    // servo Constants
    double GRASPER_WIDE_OPEN = .7;//correct
    double GRASPER_HALF_OPEN = .6; //correct
    double GRASPER_CLOSED = .4;//correct
    double AXLE_SERVO_BACK = 0; //correct
    double AXLE_SERVO_UP = .27;//change
    double AXLE_SERVO_DOWN = .7;//correct
    //double AXLE_SERVO_OUT = .4; //correct
    double GIMBAL_BASKET_SCORING = 0;//change
    double GIMBAL_SPECIMEN_SCORING = 0;//change
    double GIMBAL_RESTING_POS = .55;//change
    //double LEFT_GRASPER_OPEN = 0;//change
    //double LEFT_GRASPER_CLOSED = 0;//change
    //double RIGHT_GRASPER_OPEN = 0;//change
    //double RIGHT_GRASPER_CLOSED = 0;//change



    //end servo constants

    //motor constants
    int SLIDES_ABOVE_HIGH_RUNG = 900;//change
    int SLIDES_PUT_SP_ON_HIGH_RUNG = 80;//change
    int SLIDES_MAX = 3000;//change
    int SLIDES_MIN = 0;
    int SLIDES_TO_SUB = 60;//change

    int PIVOT_MOTOR_TO_SUB = 0;//change
    int PIVOT_MOTOR_VERTICAL = 2248;
    int PIVOT_MOTOR_HORIZONTAL = 5003;
    //end motor constants

    public MecanumDrive drive;

    DcMotor leftSlider;//from the perspective of the robot
    DcMotor rightSlider;//from the perspective of the robot
    DcMotor pivotMotor;//worm gear box
    DcMotor hangArm;//monkey arm

    //servos
    Servo grasper = null;
    Servo grasperGimbal = null;
    Servo axleRotation = null;

    Servo leftGrasper = null;//unused
    Servo rightGrasper = null;//unused
    //end servos


    public BaseRobot(HardwareMap hwMap){
        this(hwMap, new Pose2d(0,0,0));
    }

    public BaseRobot(HardwareMap hwMap, Pose2d pose){

        drive = new MecanumDrive(hwMap, pose);

        hangArm = hwMap.dcMotor.get("hangArm");


        pivotMotor = hwMap.dcMotor.get("pivotMotor");


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
        pivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        pivotMotorPos = hangArm.getCurrentPosition();

        if(pivotMotorPos > PIVOT_MOTOR_VERTICAL + 200){
            SLIDES_MAX = 400;
        }

        //servos


        grasper = hwMap.servo.get("claw");
        grasper.setPosition(GRASPER_CLOSED);

        rightGrasper = hwMap.servo.get("rightGrasper");
        //rightGrasper.setPosition(RIGHT_GRASPER_OPEN);

        leftGrasper = hwMap.servo.get("leftGrasper");//unused
        //leftGrasper.setPosition(LEFT_GRASPER_OPEN);//unused

        grasperGimbal = hwMap.servo.get("grasperGimbal");
        grasperGimbal.setPosition(GIMBAL_RESTING_POS);

        axleRotation = hwMap.servo.get("axleRotation");
        axleRotation.setPosition(AXLE_SERVO_DOWN);
        //end servos



    }



    //hang arm pos --------------
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
    //end hang arm -----------------


    //pivot motor pos ------------------
    public void updatePivotMotorPos() {
        if (pivotMotorPos < 0){

        }

        if (pivotMotorPos > PIVOT_MOTOR_HORIZONTAL + 400){
            pivotMotorPos = PIVOT_MOTOR_HORIZONTAL + 400;
        }

        pivotMotor.setTargetPosition(pivotMotorPos);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setPower(PIVOT_MOTOR_POWER);
    }
    public void setPivotMotorPos(int newPos) {
        //if (newPos < 0){
          //  newPos = 0;
        //}
        pivotMotorPos = newPos;
    }
    public void changePivotMotorPos(int deltaPos){
        pivotMotorPos += deltaPos;
    }
    //end pivot motor pos ------------------

    //gimbal pos ----------------
    public void updateGimbalPos() {
        if(gimbalPos > 1){
            gimbalPos = 1;
        }
        if(gimbalPos < 0){
            gimbalPos = 0;
        }
        grasperGimbal.setPosition(gimbalPos);

    }
    public void setGimbalPos(double newPos){
        gimbalPos = newPos;
    }
    public void changeGimbalPos(double deltaPos){
        gimbalPos += deltaPos;
    }
    //end gimbal pos ------------


    //axle servo ---------
    public void updateAxleServoPos(){
        //if(pivotMotorPos < PIVOT_MOTOR_VERTICAL && axlePos > ) {
          //  axlePos = getAXLE_SERVO_UP();
        //}
        if(axlePos > 1){
            axlePos = 1;
        }
        if(axlePos < 0){
            axlePos = 0;
        }
        axleRotation.setPosition(axlePos);
    }
    public void setAxlePos(double newPos){
        axlePos = newPos;
    }
    public void changeAxlePos(double deltaPos){
        axlePos += deltaPos;
    }
    //end axle servo ---------

    //grasper servo -------------
    public void updateGrasperPos(){

        if(grasperPos > 1){
            grasperPos = 1;
        }
        if(grasperPos < 0){
            grasperPos = 0;
        }
        grasper.setPosition(grasperPos);
    }
    public void setGrasperPos(double newPos){
        grasperPos = newPos;
    }
    public void changeGrasperPos(double deltaPos){
        grasperPos += deltaPos;
    }
    //end grasper servo ----------


    //slides -----------------
    public void updateSlidesPos() {

        // Code from teleop that grabs armPos variables and
        // Uses run to position
        if(rightSliderPos > SLIDES_MAX){
            rightSliderPos = SLIDES_MAX;
        }
        if (leftSliderPos > SLIDES_MAX){
            leftSliderPos = SLIDES_MAX;
        }
        if(rightSliderPos < SLIDES_MIN){
            rightSliderPos = SLIDES_MIN;
        }
        if (leftSliderPos < SLIDES_MIN){
            leftSliderPos = SLIDES_MIN;
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
    //end slides -----------------


    //end motor positions
    //____________________________________________________________________________________________________________________




    public void slidesUp(){
        setSlidesPos(SLIDES_MAX);
    }


    public void basketScoring(){
        setPivotMotorPos(PIVOT_MOTOR_VERTICAL);
        setSlidesPos(SLIDES_MAX);
        axleRotation.setPosition(AXLE_SERVO_UP);
        grasperGimbal.setPosition(GIMBAL_BASKET_SCORING);

    }
    public void specimenScoring(){
        pivotMotor.setTargetPosition(PIVOT_MOTOR_VERTICAL);
        setSlidesPos(SLIDES_MAX);//maybe a bit less
        axleRotation.setPosition(AXLE_SERVO_UP);
        grasperGimbal.setPosition(GIMBAL_SPECIMEN_SCORING);
    }
    public void slidesDown(){
        setSlidesPos(SLIDES_MIN);
    }
    public void reachToSub() {

        axleRotation.setPosition(AXLE_SERVO_DOWN);

        setPivotMotorPos(PIVOT_MOTOR_HORIZONTAL);
        //while(Math.abs(pivotMotor.getCurrentPosition()-PIVOT_MOTOR_HORIZONTAL)>200){

        //}
        setSlidesPos(SLIDES_TO_SUB);



        grasper.setPosition(GRASPER_WIDE_OPEN);
        grasperGimbal.setPosition(GIMBAL_RESTING_POS);
    }

    public void resetAll(){
        grasper.setPosition(GRASPER_CLOSED);
        setSlidesPos(SLIDES_MIN);
        grasperGimbal.setPosition(GIMBAL_RESTING_POS);
        axleRotation.setPosition(AXLE_SERVO_DOWN);
        setPivotMotorPos(0);
    }
    //hang arm

    public void slidesReset(){
        grasper.setPosition(GRASPER_CLOSED);
        setSlidesPos(SLIDES_MIN);
        grasperGimbal.setPosition(GIMBAL_RESTING_POS);
        axleRotation.setPosition(AXLE_SERVO_DOWN);
        setPivotMotorPos(2240);
    }
    public void slidesMax(){
        setSlidesPos(SLIDES_MAX);
    }
    //end hang arm

    //end presets
    //____________________________________________________________________________________________________________________






    // Accessors
    public int getRightSliderPos() {
        return rightSliderPos;
    }
    public int getLeftSliderPos(){
        return leftSliderPos;
    }
    public int getPivotMotorPos(){
        return pivotMotorPos;
    }
    public int getHangArmPos(){
        return hangArmPos;
    }
    public double getGimbalPos(){
        return gimbalPos;
    }
    public double getAxlePos() {
        return axlePos;
    }
    public double getGrasperPos(){
        return grasperPos;
    }
    //public double getAXLE_SERVO_OUT(){
        //return AXLE_SERVO_OUT;
    //}
    public double getAXLE_SERVO_BACK(){
        return AXLE_SERVO_BACK;
    }
    public double getAXLE_SERVO_UP(){
        return AXLE_SERVO_UP;
    }
    public double getAXLE_SERVO_DOWN(){
        return AXLE_SERVO_DOWN;
    }
    public double getGRASPER_OPEN(){
        return GRASPER_WIDE_OPEN;
    }
    public double getGRASPER_HALF_OPEN(){
        return GRASPER_HALF_OPEN;
    }
    public double getGRASPER_CLOSED(){
        return GRASPER_CLOSED;
    }
    public double getGIMBAL_BASKET_SCORING(){
        return GIMBAL_BASKET_SCORING;
    }
    public double getGIMBAL_SPECIMEN_SCORING(){
        return GIMBAL_SPECIMEN_SCORING;
    }
    public double getGIMBAL_RESTING_POS(){
        return GIMBAL_RESTING_POS;
    }

    public double getRIGHT_SLIDE_POWER(){
        return RIGHT_SLIDE_POWER;
    }
    public double getLEFT_SLIDE_POWER(){
        return LEFT_SLIDE_POWER;
    }
    public double getPIVOT_MOTOR_POWER(){
        return PIVOT_MOTOR_POWER;
    }

    public int getSLIDES_ABOVE_HIGH_RUNG(){
        return SLIDES_ABOVE_HIGH_RUNG;
    }
    public int getSLIDES_PUT_SP_ON_HIGH_RUNG(){
        return SLIDES_PUT_SP_ON_HIGH_RUNG;
    }

    public int getSLIDES_MAX(){
        return SLIDES_MAX;
    }

    public int getPIVOT_MOTOR_HORIZONTAL(){
        return PIVOT_MOTOR_HORIZONTAL;
    }
    public int getPIVOT_MOTOR_VERTICAL(){
        return PIVOT_MOTOR_VERTICAL;
    }


    //end accessors
    //____________________________________________________________________________________________________________________

    public void delay(double seconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < seconds){

        }

    }


}

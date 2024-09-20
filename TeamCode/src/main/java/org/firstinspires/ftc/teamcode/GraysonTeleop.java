package org.firstinspires.ftc.teamcode;

//import com.google.blocks.ftcrobotcontroller.runtime.CRServoAccess;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class GraysonTeleop extends LinearOpMode {

    BaseRobot robot;

    int leftSliderPos = 0;
    int rightSliderPos = 0;

    int hangArmPos = 0;

    int pivotMotorPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {


        robot = new BaseRobot(hardwareMap);
    }

}

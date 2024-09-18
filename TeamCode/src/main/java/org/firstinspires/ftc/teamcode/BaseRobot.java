package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BaseRobot {

    DcMotor leftSlider;
    DcMotor rightSlider;
    DcMotor pivotMotor;//worm gear box
    DcMotor hangArm;//monkey arm



    public BaseRobot(HardwareMap hwMap){

        hangArm = hwMap.dcMotor.get("hangArm");
        hangArm.setPower(1);

        pivotMotor = hwMap.dcMotor.get("pivotMotor");
        pivotMotor.setPower(1);

        leftSlider = hwMap.dcMotor.get("leftSlider");
        leftSlider.setPower(1);

        leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlider.setTargetPosition(100);
        leftSlider.setPower(1);

    }


    public void sliderRunTo(int position){
        leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlider.setTargetPosition(position);
        rightSlider.setTargetPosition(position);
        leftSlider.setPower(1);
        rightSlider.setPower(1);



    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Claw {
    LinearOpMode opMode;
    Servo leftServo;
    Servo rightServo;
    DcMotor clawSpinnerMotor = null;
    
    public Claw(){
        
    }
    public void init(LinearOpMode opMode){
        this.opMode = opMode;
        leftServo = opMode.hardwareMap.get(Servo.class, "cl");
        rightServo = opMode.hardwareMap.get(Servo.class, "cr");
        clawSpinnerMotor = opMode.hardwareMap.get(DcMotor.class, "csm");
    }
    // Close the claw and grab the freight
    public void close(){
        leftServo.setPosition(Cals.kClawLeftClosePos);
        rightServo.setPosition(Cals.kClawRightClosePos);
        clawSpinnerMotor.setPower(0.0);
    }
    // Open the claw and release the freight
    public void open(){
        leftServo.setPosition(Cals.kClawLeftOpenPos);
        rightServo.setPosition(Cals.kClawRightOpenPos);
        LevelEnum level = Darth.arm.getLevel();
        if(level == LevelEnum.L3 || level == LevelEnum.L2 || level == LevelEnum.L1){
            clawSpinnerMotor.setPower(1.0);
        }else if(level == LevelEnum.DOWN){
            clawSpinnerMotor.setPower(-1.0);
        }else {
            clawSpinnerMotor.setPower(0.0);
        }
    }
    

  

}
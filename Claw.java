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
    
    public Claw(){
        
    }
    public void init(LinearOpMode opMode){
        this.opMode = opMode;
        leftServo = opMode.hardwareMap.get(Servo.class, "cl");
        rightServo = opMode.hardwareMap.get(Servo.class, "cr");
    }
    public void close(){
        leftServo.setPosition(Cals.kClawLeftClosePos);
        rightServo.setPosition(Cals.kClawRightClosePos);
    }
    public void open(){
        leftServo.setPosition(Cals.kClawLeftOpenPos);
        rightServo.setPosition(Cals.kClawRightOpenPos);
    }
    

  

}
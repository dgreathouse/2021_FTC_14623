package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Spinner {
    LinearOpMode opMode;
    
    DcMotor leftSpinnerMotor;
    DcMotor rightSpinnerMotor;
    
    ElapsedTime runtime = new ElapsedTime();
    public Spinner(){
        
    }
    public void init(LinearOpMode opMode){
        this.opMode = opMode;
        
        leftSpinnerMotor = opMode.hardwareMap.dcMotor.get("ls");
        rightSpinnerMotor = opMode.hardwareMap.dcMotor.get("rs");
        
        leftSpinnerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSpinnerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSpinnerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSpinnerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
    }
    public void spin(double leftSpeed, double rightSpeed){
        leftSpinnerMotor.setPower(leftSpeed);
        rightSpinnerMotor.setPower(rightSpeed);
    }
    public void spin(boolean leftSpeed, boolean rightSpeed){
        if(leftSpeed)
            leftSpinnerMotor.setPower(1);
        else 
            leftSpinnerMotor.setPower(0);
        
        if(rightSpeed)
            rightSpinnerMotor.setPower(1);
        else 
            rightSpinnerMotor.setPower(0);
            
    }
    public void spinLeft(double timeOut){
        runtime.reset();
        while(runtime.seconds() < timeOut){
            leftSpinnerMotor.setPower(1.0);
            Darth.chassis.drive(0.1,0);
        }
        leftSpinnerMotor.setPower(0.0);
    }

    public void spinRight(double timeOut){
        runtime.reset();
        while(runtime.seconds() < timeOut){
            rightSpinnerMotor.setPower(1.0);
            Darth.chassis.drive(0.1,0);
        }
        rightSpinnerMotor.setPower(0.0);
    }
    
}
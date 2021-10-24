package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotor.RunMode;


public class Arm {
 
  HardwareMap HwMap;
  DcMotorEx armMotor = null;
  PIDFCoefficients pid;
  LinearOpMode opMode;
  // Class constructor
  public Arm(){
    
  }
  // Initialize the class 
  public void init(LinearOpMode opMode){
    this.opMode = opMode;
    armMotor = (DcMotorEx)opMode.hardwareMap.dcMotor.get("arm");
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    pid = new PIDFCoefficients(20.0,0.0,0.0,0.0);
    armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,pid);
    
    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armMotor.setTargetPositionTolerance(1);
    
  }
  // This method will move the arm to the requested motor position in counts.
  public void rotateToPos(int counts, double power){
    armMotor.setTargetPosition(counts);
    armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    armMotor.setPower(power);
   // opMode.telemetry.addData("Arm Counts","%d Cnts", armMotor.getCurrentPosition()); 
    //opMode.telemetry.update();
  }
  
  public void rotateWithPower(double pwr){
    armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    armMotor.setPower(pwr);
  }

  public int getPosition(){
    return armMotor.getCurrentPosition();
    
  }
  public void rotateToLevel(int level, double power){
    int pos = 0;
    if(level == 0){pos = 0;}else 
    if(level == 1){pos = Cals.kArmL1Pos;}else 
    if(level == 2){pos = Cals.kArmL2Pos;}else
    if(level == 3){pos = Cals.kArmL3Pos;}else
    pos = Cals.kArmUpPos;
    //armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    armMotor.setTargetPosition(pos);
    armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    armMotor.setPower(power);
  }
  

}
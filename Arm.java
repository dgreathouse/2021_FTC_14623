package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.logging.Level;
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
  LevelEnum level;
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

  public int getPosition(){
    return armMotor.getCurrentPosition();
  }
  
  public LevelEnum getLevel(){
    return level;
  }
  /// <summary>
  /// Rotate the arm to a certain level
  /// </summary>
  /// <param name="level">0 = Front of robot and down to pick up freight
  ///                     1 = Level 1 of shipping hub
  ///                     2 = Level 2 of shipping hub
  ///                     3 = Level 3 of shipping hub
  ///                     4 = Arm straight up for driving around</param>
  ///                     5 = NO no change from buttons
  /// <param name="power">The power to drive at. 0.5 is a good speed.</param>
  public void rotateToLevel(LevelEnum level, double power){
    int pos = 0;
    this.level = level;
    if(level == LevelEnum.DOWN){pos = 0;}else 
    if(level == LevelEnum.L1){pos = Cals.kArmL1Pos;}else 
    if(level == LevelEnum.L2){pos = Cals.kArmL2Pos;}else
    if(level == LevelEnum.L3){pos = Cals.kArmL3Pos;}else
    if(level == LevelEnum.UP){pos = Cals.kArmUpPos;}
    
    if(level != LevelEnum.NO){
      
      armMotor.setTargetPosition(pos);
      armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
      armMotor.setPower(power);
    }
  }
  

}
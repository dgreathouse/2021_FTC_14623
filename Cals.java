package org.firstinspires.ftc.teamcode;


public class Cals {
 
 // Arm Cals
 public static final int kArmDownPos = 0;
 public static final int kArmUpPos = -1750;
 public static final int kArmL3Pos = -2400;
 public static final int kArmL2Pos = -2850;
 public static final int kArmL1Pos = -3250;
 public static final double kArmPower = 1.0;
 
 // Distance Sensor Cals
 public static final double kDisSensorServoStraight = 0.48;
 public static final double kDisSensorServoPosOffset = 0.09;
 public static final double kDisSensorServoPos2R = kDisSensorServoStraight - kDisSensorServoPosOffset;
 public static final double kDisSensorServoPos2L = kDisSensorServoStraight + kDisSensorServoPosOffset;
 // Chassis Cals 
 public static final double kChassisCountsPerMotorRev = 480; // 60:1 TorqueNado
 public static final double kChassisDriveGearReduction = 1.0;
 public static final double kChassisWheelDiameter = 4.357;
 //public static final double kChassisRotationCountsPerDegree = 1.3611;
 
 public static final double kChassisCountsPerInch = (kChassisCountsPerMotorRev * kChassisDriveGearReduction) / 
  (kChassisWheelDiameter * 3.1415);
 public static final double kChassisAutoDriveSpeed = 0.5;
 public static final double kChassisAutoTurnSpeed = 0.5;
 public static final int kChassisAutoTolerance = 5;
  
 // Claw
 public static final double kClawRightOpenPos = 0.55;
 public static final double kClawLeftOpenPos = 0.65;
 public static final double kClawRightClosePos = 0.4;
 public static final double kClawLeftClosePos = 0.75;
 
}

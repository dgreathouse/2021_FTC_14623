package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.logging.Level;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="4 Wheel 2021.2", group="Linear Opmode") 

public class TeleOp_2021_1 extends LinearOpMode{
 /* Declare OpMode members. */
 private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
 
 double ChassisDrive;
 double ChassisRotate;
 boolean SpinnerLeft;
 boolean SpinnerRight;
 boolean ArmDown;
 boolean ArmL1;
 boolean ArmL2;
 boolean ArmL3;
 boolean ArmUp;
 boolean ClawClose;
 boolean ClawOpen;
 LevelEnum ArmPos;
 
 
 // operational constants
 double joyScale = 1;
 
 @Override
 public void runOpMode(){
  telemetry.addData("Status", "Initialized");
  telemetry.update();
  
  //Darth robot = new Darth();
  
  Darth.init(this);
  waitForStart();
  runtime.reset();
  
  while(opModeIsActive()) {
   telemetry.addData("Status", "Run Time: %.2f" + runtime.seconds());
   
   // Drive Chassis
   //ChassisDrive = gamepad1.left_trigger + gamepad2.left_trigger;               // Left Forward Right Reverse
   //ChassisDrive = ChassisDrive - gamepad1.right_trigger - gamepad2.right_trigger; // Left Forward Right Reverse
   
   ChassisDrive = gamepad1.right_trigger + gamepad2.right_trigger;               // right Forward Right Reverse
   ChassisDrive = ChassisDrive - gamepad1.left_trigger - gamepad2.left_trigger; // right Forward Right Reverse
   ChassisRotate = gamepad1.right_stick_x + gamepad2.right_stick_x;
   
   Darth.chassis.drive(ChassisDrive,ChassisRotate);
   
   // Rotate Spinners
   SpinnerLeft = gamepad1.start || gamepad2.start;
   SpinnerRight = gamepad1.back || gamepad2.back;
   
   Darth.spinner.spin(SpinnerLeft,SpinnerRight);
   
   ArmPos = LevelEnum.NO;
   // Move Arm
   ArmDown = gamepad1.x || gamepad2.x;
   ArmL3 = gamepad1.y || gamepad2.y;
   ArmL2 = gamepad1.b || gamepad2.b;
   ArmL1 = gamepad1.a || gamepad2.a;
   
   ArmUp = gamepad1.left_stick_button || gamepad2.left_stick_button;
   
   if(ArmUp){ArmPos = LevelEnum.UP;}else
   if(ArmL1){ArmPos = LevelEnum.L1;}else
   if(ArmL3){ArmPos = LevelEnum.L3;}else 
   if(ArmL2){ArmPos = LevelEnum.L2;}else
   if(ArmDown){ArmPos = LevelEnum.DOWN;}

   Darth.arm.rotateToLevel(ArmPos,Cals.kArmPower);
   
   // Open/Close Claw
   ClawClose = gamepad2.left_bumper || gamepad1.left_bumper;
   ClawOpen = gamepad2.right_bumper || gamepad1.right_bumper;
   
   if(ClawClose){
     Darth.claw.close(); 
   }else if(ClawOpen){
     Darth.claw.open();
   }
   
   // Send some telemetry data to the drive station.
   // Most telemetry is done in the classes for the subsystem

   //telemetry.addData("Arm Motor", "Input (%d), Pos (%d)", ArmPos, Darth.arm.getPosition());
   telemetry.addData("Dis Sensor","%.2f In", Darth.disSensor.getDistance()); 
   telemetry.addData("Chassis", "Driven Inches = %.2f", Darth.chassis.getDistanceInches());
  //elemetry.addData("Chassis", "LeftVel = %.2f,  RightVel = %.2f", Darth.chassis.leftRearMotor.getVelocity(AngleUnit.DEGREES), Darth.chassis.rightRearMotor.getVelocity(AngleUnit.DEGREES));
   telemetry.addData("Chassis","%.2f Deg", Darth.imuInt.getAngle()); 
   //PIDCoefficients pid = Darth.chassis.rightRearMotor.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
   //telemetry.addData("Chassis", "P = %.2f,  I = %.2f, D = %.2f", pid.p, pid.i, pid.d);

   telemetry.update();
  }
 }
}

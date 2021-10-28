package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.logging.Level;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Right ShippingHub->Spinner->StorageUnit", group="Blue")

public class AutoBlue_R_SH_SP_SU extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    LevelEnum level = LevelEnum.UP;
    
    @Override
    public void runOpMode() {
        Darth.init(this);
        Arm arm = Darth.arm;
        Chassis chassis = Darth.chassis;
        Claw claw = Darth.claw;
        DisSensor disSensor = Darth.disSensor;
        Spinner spinner = Darth.spinner;
        double levelDistance = -19;
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        
        while(opModeIsActive()) {
            level = disSensor.getLevel(LevelEnum.L1);                           // Get level from distance sensor
            levelDistance = disSensor.getLevelDistance(level);                  // Calculate the distance to travel for the given level
            claw.close();                                                       // Claw Close
            chassis.driveToInches(-8.0, 450.0, 3.0);                            // Chassis Drive 
            arm.rotateToLevel(LevelEnum.UP, 0.5);                               // Arm Rotate 0=Down, 1=L1, 2=L2, 3=L3, 4=Up
            chassis.rotateToAngle(90.0,450.0, 5.0);                             // Chassis Rotate 
            chassis.driveToInches(-22.0, 450.0, 3.0);                           // Chassis Drive 
            arm.rotateToLevel(level, 0.5);                                      // Arm Rotate (To level of team shipping element)
            chassis.rotateToAngle(0.0,450.0, 5.0);                              // Chassis Rotate 
            chassis.driveToInches(levelDistance, 450.0, 5.0);                   // Chassis Drive (Calculated distance to level)
            claw.open();                                                        // Claw Open
            runtime.reset(); while(runtime.seconds()<= 1.0){}                   // Delay for freight to fall
            chassis.driveToInches(-levelDistance-5, 450.0, 5.0);                // Chassis Drive (Calculated distance to level back)
            arm.rotateToLevel(LevelEnum.UP, 0.5);                               // Arm Rotate (To up position which is level 4)
            chassis.rotateToAngle(80.0,450.0, 5.0);                             // Chassis Rotate
            chassis.driveToInches(34.0, 450.0, 3.0);                            // Chassis Drive (to wall by spinner)
            spinner.spinRight(3.0,0.1,0.0);                                     // Spinner Spin (Right)
            stop();                                                             // Stop the autonomous mode
        }
     }    
 }
 /* Method Doc
 chassis.driveToInches(Inches, Velocity, timeOut)
 chassis.drive(drv,rot,timeOut)
 chassis.rotateToAngle(Angle(Deg), Velocity, timeout)
 armRotateToLevel(Level to rotate to) Enum values from LevelEnum (DOWN,UP,L1,L2,L3)
 spinner.spinRight(Seconds to spin, Chassis Drive power, Chassis Rotate power)
 spinner.spinLeft(Seconds to spin, Chassis Drive power, Chassis Rotate power) 
 claw.open()
 claw.close()
 
 Delay
 runtime.reset(); while(runtime.seconds()<= 1.0){}  Change the "1.0" to the seconds needed for the delay
 */
 
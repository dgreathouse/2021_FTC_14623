package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Right ShippingHub->Spinner->StorageUnit", group="Blue")

public class AutoBlue_R_SH_SP_SU extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    int level = 4;
    
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
            level = disSensor.getLevel(1);                                      // Get level from distance sensor
            levelDistance = levelDistance + (3-level)* 1.5;                     // Calculate the distance to travel for the given level
            claw.close();                                                       // Claw Close
            chassis.driveToInches(-8.0, 450.0, 3.0);                            // Chassis Drive 
            arm.rotateToLevel(4, 0.5);                                          // Arm Rotate 0=Down, 1=L1, 2=L2, 3=L3, 4=Up
            chassis.rotateToAngle(90.0,450.0, 5.0);                             // Chassis Rotate 
            chassis.driveToInches(-22.0, 450.0, 3.0);                           // Chassis Drive 
            arm.rotateToLevel(level, 0.5);                                      // Arm Rotate (To level of team shipping element)
            chassis.rotateToAngle(0.0,450.0, 5.0);                              // Chassis Rotate 
            chassis.driveToInches(levelDistance, 450.0, 5.0);                   // Chassis Drive (Calculated distance to level)
            claw.open();                                                        // Claw Open
            runtime.reset(); while(runtime.seconds()<= 1.0){}                   // Delay 
            chassis.driveToInches(-levelDistance-5, 450.0, 5.0);                // Chassis Drive (Calculated distance to level back)
            arm.rotateToLevel(4, 0.5);                                          // Arm Rotate (To up position which is level 4)
            chassis.rotateToAngle(80.0,450.0, 5.0);                             // Chassis Rotate
            chassis.driveToInches(34.0, 450.0, 3.0, true);                      // Chassis Drive (to wall by spinner)
            spinner.spinRight(3.0);                                             // Spinner Spin (Right)

            stop();                                                             // Stop the auto?nomous mode
        }
     }    
 }
 /* Method Doc
 chassis.driveToInches(Inches, Velocity, timeOut)
 chassis.rotateToAngle(Angle(Deg), Velocity, timeout)
 armRotateToLevel(Level to rotate to) 0=Down, 1=L1, 2=L2, 3=L3, 4=Up
 spinner.spinRight(Seconds to spin)  This also drives forward at low power
 spinner.spinLeft(Seconds to spin)  This also drives forward at low power
 */
 
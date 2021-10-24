package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Right Spinner->ShippingHub->StorageUnit", group="Blue")

public class AutoBlue_R_SP_SH_SU extends LinearOpMode {
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

            stop();
        }
        // Stop all motors if needed
    }    

}
 /* Method Doc
 chassis.driveToInches(Inches, Velocity, timeOut)
 chassis.rotateToAngle(Angle(Deg), Velocity, timeout)
 armRotateToLevel(Level to rotate to) 0=Down, 1=L1, 2=L2, 3=L3, 4=Up
 spinner.spinRight(Seconds to spin)  This also drives forward at low power
 spinner.spinLeft(Seconds to spin)  This also drives forward at low power
 */
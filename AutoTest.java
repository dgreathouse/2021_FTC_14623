package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto Test", group="_Test")

public class AutoTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
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
         //   level = disSensor.getLevel(3);                                      // Get level from distance sensor
         //   levelDistance = levelDistance + (3-level)* 1.5;                     // Calculate the distance to travel for the given level
         //   claw.close();                                                       // Claw Close
            chassis.rotateToAngle(-90.0, 550.0, 5.0);
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
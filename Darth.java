package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Darth {

    public static Chassis chassis = new Chassis();
    public static Spinner spinner = new Spinner();
    public static Arm arm = new Arm();
    public static Claw claw = new Claw();
    public static DisSensor disSensor= new DisSensor();
    public static IMUInternal imuInt = new IMUInternal();
    
    public Darth(){
        
    }
    
    public static void init(LinearOpMode linearOpMode){
        chassis.init(linearOpMode); 
        arm.init(linearOpMode);
        disSensor.init(linearOpMode);
        claw.init(linearOpMode);
        spinner.init(linearOpMode);
        imuInt.init(linearOpMode);
    }
    
}

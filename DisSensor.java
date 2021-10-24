package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import java.util.logging.Level;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class DisSensor {

    DistanceSensor ds;
    Servo dsServo;
    LinearOpMode opMode;
    boolean isLevelFound = false;
    public int level = 1;
    int searchingLevel = 1;
    double position;
    double distance = 0.0;
    private ElapsedTime runtime = new ElapsedTime();
    public void init(LinearOpMode opMode){
        
        this.opMode = opMode;
        ds = opMode.hardwareMap.get(DistanceSensor.class, "ds");
        dsServo = opMode.hardwareMap.get(Servo.class, "dss");
    }
    public double getDistance(){
        return ds.getDistance(DistanceUnit.INCH);
        
    }
    public double getDistanceAverage(int averages){
        double rtn = 0;
        for(int i = 0; i < averages; i++){
            rtn = rtn + getDistance();
        }
        return rtn / averages;
    }
    public void setDisSensorServo(double pos){
        position = pos;
        dsServo.setPosition(pos);
    }
    public double getPosition(){
        return dsServo.getPosition();
    }

    /*
        Return the level the Team Shipping Element is at for the start of Auto
    */
    public int getLevel(int startingLevel){
        double locationOfLevel2 = Cals.kDisSensorServoPos2R;
        if(startingLevel == 3){
            locationOfLevel2 = Cals.kDisSensorServoPos2L;
        }
        // Set the servo to postion 1. This should have been where it was set before the match.
        setDisSensorServo(Cals.kDisSensorServoStraight);
        // reset the timer and wait 2 seconds for servo to get to the new position
        runtime.reset(); while(runtime.seconds() <= 1.0){}
        // Get 8 samples from the sensor and average them.
        distance = getDistanceAverage(8);
 
        // If the averaged distance is less than 50 inches then return level 1
        if(distance < 25 && distance > 10) {
            return startingLevel;
        }
        // Set the servo to postion 2
        setDisSensorServo(locationOfLevel2);
        // reset the timer and wait 2 seconds for servo to get to the new position
        runtime.reset(); 
        while(runtime.seconds() <= 2.0){}
        // Reset the distance variable
        distance = 0; 
        // Get 8 samples from the sensor and average them.
        distance = getDistanceAverage(8);
        // If the averaged distance is less than 50 inches then return level 2
        if(distance < 25 && distance > 10) {
            return 2;
        }else { // else we will assume it is level 3 and return 3
            if(startingLevel == 3){
                return 1;
            }else {
                return 3;    
            }
            
        }
    }
}

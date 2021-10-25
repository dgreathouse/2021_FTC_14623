package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Chassis {

    public DcMotorEx leftRearMotor = null;
    public DcMotorEx rightRearMotor = null;
    BNO055IMU imu;
    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime pidTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    // declare motor speed variables
    double RR;
    double LR;
    // declare joystick position variables
    double Drive, Rotate;
    LinearOpMode opMode;
    PIDFCoefficients pidDrive;
    PIDFCoefficients pidRotate;
    double motorMax = 1; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode

    public Chassis() {
    }

    public void init(LinearOpMode opMode) {
        this.opMode = opMode;

        /*
         * Initialize the hardware variables. Note that the strings used here as
         * parameters to 'get' must correspond to the names assigned during the robot
         * configuration step (using the FTC Robot Controller app on the phone).
         */
        leftRearMotor = (DcMotorEx) opMode.hardwareMap.dcMotor.get("l");
        rightRearMotor = (DcMotorEx) opMode.hardwareMap.dcMotor.get("r");
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");

        pidDrive = new PIDFCoefficients(1.0, 0.20, 0.0, 25.0);
        pidRotate = new PIDFCoefficients(1.0, 0.20, 0.0, 25.0);
        

        // Set the drive motor direction:
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);

        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void rotateToAngle(double angle, double velocity, double timeOut) {
        boolean isBusy = true;
        double val = 0.0;
        PIDController pid = new PIDController(3.50, 0.02, 0.0);

        pid.setOutputRange(140.0, velocity);
        pid.setInputRange(0.0, 180.0);
        pid.setTolerance(0.5); // 0.5 = 0.9Deg
        pid.setSetpoint(angle);
        pid.enable();
        runtime.reset(); pidTimer.reset();
        
        while (isBusy && runtime.seconds() < timeOut) {
            val = pid.performPID(getAngle());
             opMode.telemetry.addData("PID ", "val = (%.2f)", val);
             opMode.telemetry.addData("Gyro ", "Deg = (%.2f)", Darth.imuInt.getAngle());
             opMode.telemetry.update();
            rotateAtVelocity(val);
            if (pid.onTarget(16)) {
                isBusy = false;
            }
            while(pidTimer.seconds() < 0.020){} pidTimer.reset();
        }
        pid.disable();
        disableMotors();
        opMode.telemetry.addData("Gyro ", "Deg = (%.2f)", Darth.imuInt.getAngle());
        opMode.telemetry.update();
    }

    public void driveToInches(double inches, double velocity, double timeOut) {
        boolean isBusy = true;
        double val = 0.0;
        PIDController pid = new PIDController(17.20, 0.05, 0.0);
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pid.setOutputRange(130.0, velocity);
        pid.setInputRange(0.0, 72.0);
        pid.setTolerance(0.50); // 0.5 = 0.36 inch
        pid.setSetpoint(inches);
        pid.enable();
        runtime.reset(); pidTimer.reset();

        while (isBusy && runtime.seconds() < timeOut) {
            val = pid.performPID(getDistanceInches());
            opMode.telemetry.addData("PID ", "val = (%.2f)", val);
            opMode.telemetry.addData("Dis ", "In = (%.2f)", getDistanceInches());
            opMode.telemetry.addData("Time ", "ms = (%.2f)", runtime.seconds());
            //opMode.telemetry.addData("Time ", "ms = (%.2f)", pidTimer.seconds());
            opMode.telemetry.update();

            driveAtVelocity(val);
            if (pid.onTarget(16)) {
                isBusy = false;
            }
            while(pidTimer.milliseconds() < 20){} pidTimer.reset();
        }
        pid.disable();
        disableMotors();
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // opMode.telemetry.addData("Dis ", "in = (%.2f)", getDistanceInches());
        // opMode.telemetry.update();
    }

    public void drive(double drv, double rot) {
        double lr, rr = 0;

        // Calculate Motor values
        lr = drv + rot;
        rr = drv - rot;

        // Clip motor power values to +-motorMax
        lr = Range.clip(lr, -motorMax, motorMax);
        rr = Range.clip(rr, -motorMax, motorMax);

        // Send values to the motors
        leftRearMotor.setPower(lr);
        rightRearMotor.setPower(rr);
    }
    public void drive(double drv, double rot, double timeOut) {
        double l, r = 0;

        // Calculate Motor values
        l = drv + rot;
        r = drv - rot;

        // Clip motor power values to +-motorMax
        l = Range.clip(l, -motorMax, motorMax);
        r = Range.clip(r, -motorMax, motorMax);
        
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    
    
        runtime.reset();
        while (runtime.seconds() < timeOut) {
            leftRearMotor.setPower(l);
            rightRearMotor.setPower(r);
        }
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);

    }
    public void driveAtVelocity(double velocity) {
        leftRearMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidDrive);
        rightRearMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidDrive);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setVelocity(velocity, AngleUnit.DEGREES);
        rightRearMotor.setVelocity(velocity, AngleUnit.DEGREES);
    }

    public void rotateAtVelocity(double velocity) {
        leftRearMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidRotate);
        rightRearMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidRotate);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setVelocity(-velocity, AngleUnit.DEGREES);
        rightRearMotor.setVelocity(velocity, AngleUnit.DEGREES);
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        leftRearMotor.setMode(mode);
        rightRearMotor.setMode(mode);
    }

    public double getAngle() {
        return Darth.imuInt.getAngle();
    }

    public double getDistanceInches() {
        return getDistance() / Cals.kChassisCountsPerInch;
    }

    public double getDistance() {
        return (leftRearMotor.getCurrentPosition() + rightRearMotor.getCurrentPosition()) / 2.0;
    }

    public int getLeftMotorPosition() {
        return leftRearMotor.getCurrentPosition();
    }

    public int getRightMotorPosition() {
        return rightRearMotor.getCurrentPosition();
    }

    private void disableMotors() {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setPower(0.0);
        rightRearMotor.setPower(0.0);
    }
}

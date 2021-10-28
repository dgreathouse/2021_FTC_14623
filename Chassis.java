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
    public DcMotorEx motBackLeft = null;
    public DcMotorEx motBackRight = null;
    public DcMotorEx motFrontLeft = null;
    public DcMotorEx motFrontRight = null;

    // public DcMotorEx leftRearMotor = null;
    // public DcMotorEx rightRearMotor = null;
    // public DcMotorEx leftFrontMotor = null;
    // public DcMotorEx rightFrontMotor = null;
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

        motBackLeft = (DcMotorEx) opMode.hardwareMap.dcMotor.get("lr");
        motBackRight = (DcMotorEx) opMode.hardwareMap.dcMotor.get("rr");
        motFrontLeft = (DcMotorEx) opMode.hardwareMap.dcMotor.get("lf");
        motFrontRight = (DcMotorEx) opMode.hardwareMap.dcMotor.get("rf");
        
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        
        // PIDF values are for velocity control of the motor controller
        // A zero for F term may allow a better velocity control using Legacy PID
        pidDrive = new PIDFCoefficients(1.0, 0.20, 0.0, 0.0);
        pidRotate = new PIDFCoefficients(1.0, 0.20, 0.0, 0.0);
        

        // Set the drive motor direction:
        motBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motBackRight.setDirection(DcMotor.Direction.REVERSE);
        motFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motFrontRight.setDirection(DcMotor.Direction.REVERSE);
        
        motBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        motBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    /// <summary>
    /// Rotate the robot
    /// </summary>
    /// <param name="angle">The angle in degrees to rotate to. Zero angle is 
    ///                     from when the robot was turned on.
    ///                     CW is minus, CCW is plus. </param>
    /// <param name="velocity">The velocity to rotate at. Range is 150-750</param>
    /// <param name="timeOut">The time to disable this call if not complete with reaching the angle</param>
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
    /// <summary>
    /// Drive the robot to a set Inches
    /// </summary>
    /// <param name="inches">The distance in inches to drive to. Every call to 
    ///                      this method will reset the encoders to 0. Therefore
    ///                      inhes is always starting at 0. </param>
    /// <param name="velocity">The velocity to travel at. Range is 150-750</param>
    /// <param name="timeOut">The time to disable this call if not complete with reaching the distance</param>
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
            while(pidTimer.seconds() < 0.020){} pidTimer.reset();
        }
        pid.disable();
        disableMotors();
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    /// <summary>
    /// Drive the robot with inputs from the game pad. 
    /// This is used in TeleOp. 
    /// </summary>
    /// <param name="drv">Range -1 to 1 Drive reverse and forward </param>
    /// <param name="rot">Range -1 to 1 Rotate CCW and CW</param>
    public void drive(double drv, double rot) {
        double bl, br = 0;
        double fl, fr = 0;
        // Calculate Motor values
        bl = drv + rot;
        br = drv - rot;
        fl = bl;
        fr = br;

        // Clip motor power values to +-motorMax
        bl = Range.clip(bl, -motorMax, motorMax);
        br = Range.clip(br, -motorMax, motorMax);
        fl = Range.clip(fl, -motorMax, motorMax);
        fr = Range.clip(fr, -motorMax, motorMax);

        // Send values to the motors
        motBackLeft.setPower(bl);
        motBackRight.setPower(br);
        motFrontLeft.setPower(fl);
        motFrontRight.setPower(fr);
    }
        /// <summary>
    /// This method should be used only in autonomous when a simple drive
    /// at a power for a time is needed. Such as driving in to the warehouse
    /// over the bumps to make sure the robot crosses the lines.
    /// </summary>
    /// <param name="drv">Range -1 to 1 Drive reverse and forward </param>
    /// <param name="rot">Range -1 to 1 Rotate CCW and CW</param>
    /// <param name="timeOut">The time to stop driving the robot. </param>
    public void drive(double drv, double rot, double timeOut) {
        
        double bl, br = 0;
        double fl, fr = 0;
        // Calculate Motor values
        bl = drv + rot;
        br = drv - rot;
        fl = bl;
        fr = br;

        // Clip motor power values to +-motorMax
        bl = Range.clip(bl, -motorMax, motorMax);
        br = Range.clip(br, -motorMax, motorMax);
        fl = Range.clip(fl, -motorMax, motorMax);
        fr = Range.clip(fr, -motorMax, motorMax);

        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        runtime.reset();
        while (runtime.seconds() < timeOut) {
            motBackLeft.setPower(bl);
            motBackRight.setPower(br);
            motFrontLeft.setPower(fl);
            motFrontRight.setPower(fr);
        }
        disableMotors();
    }
    private void driveAtVelocity(double velocity) {
        motBackLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidDrive);
        motBackRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidDrive);
        motFrontLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidDrive);
        motFrontRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidDrive);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motBackLeft.setVelocity(velocity, AngleUnit.DEGREES);
        motBackRight.setVelocity(velocity, AngleUnit.DEGREES);
        motFrontLeft.setVelocity(velocity, AngleUnit.DEGREES);
        motFrontRight.setVelocity(velocity, AngleUnit.DEGREES);
    }

    private void rotateAtVelocity(double velocity) {
        motBackLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidRotate);
        motBackRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidRotate);
        motFrontLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidRotate);
        motFrontRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidRotate);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motBackLeft.setVelocity(-velocity, AngleUnit.DEGREES);
        motBackRight.setVelocity(velocity, AngleUnit.DEGREES);
        motFrontLeft.setVelocity(-velocity, AngleUnit.DEGREES);
        motFrontRight.setVelocity(velocity, AngleUnit.DEGREES);
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        motBackLeft.setMode(mode);
        motBackRight.setMode(mode);
        motFrontLeft.setMode(mode);
        motFrontRight.setMode(mode);
    }

    public double getAngle() {
        return Darth.imuInt.getAngle();
    }

    public double getDistanceInches() {
        return getDistance() / Cals.kChassisCountsPerInch;
    }

    public double getDistance() {
        return (motBackLeft.getCurrentPosition() + motBackRight.getCurrentPosition()) / 2.0;
    }

    public int getLeftMotorPosition() {
        return (motBackLeft.getCurrentPosition() + motFrontLeft.getCurrentPosition())/2;
    }

    public int getRightMotorPosition() {
        return (motBackRight.getCurrentPosition() + motFrontRight.getCurrentPosition())/2;
    }

    public void disableMotors() {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motBackLeft.setPower(0.0);
        motBackRight.setPower(0.0);
        motFrontLeft.setPower(0.0);
        motFrontRight.setPower(0.0);
    }
}
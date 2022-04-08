package frc.robot.autonomous;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;




public class Drive{
    
    double P;
    double I;
    double D;
    double integral, previous_error, setpoint = 0;
    ADXRS450_Gyro gyro;
    DifferentialDrive robotDrive;
    double rcw;
    Drivetrain drivetrain;


    public Drive(ADXRS450_Gyro gyro, Drivetrain drivetrain) {
        this.gyro = gyro;
        this.drivetrain = drivetrain;
    }

    public void setSetpoint(int setpoint) {
        this.setpoint = setpoint;
    }

    /*
    public void changePIDValues() {
        P = SmartDashboard.getNumber("Drive P", 0.225);
        I = SmartDashboard.getNumber("Drive I", 0.0675);
        D = SmartDashboard.getNumber("Drive D", 0.0);
    } */

    public void PID() {
        double realAngle = (gyro.getAngle() / 150) * 360;
        //double P = 0.05;
       
        double error = 60 - gyro.getAngle(); // Error = Target - Actual
        this.integral += (error * .02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
        double derivative = (error - this.previous_error) / .02;
        System.out.println("P: " + P);
        System.out.println("I: " + I);
        System.out.println("D: " + D);

        rcw = P * error + I * this.integral + D * derivative;
        System.out.println("rcw: " + rcw);

    }

    public void execute(DifferentialDrive robotDrive)
    {
        System.out.println("rcw: " + rcw);
        PID();
        System.out.println("Executing...");
        robotDrive.arcadeDrive(0, rcw);
    }

    /**Converts from WPI's 0-300 angle system to a normal 0-360 angle */
    public double convertToRealAngle(double angle) {
        return ((angle / 300) * 360);
    }

    public double getRealAngle() {
        return ((gyro.getAngle() / 300) * 360);
    }

    public void printAngle() {
        //System.out.println("angle from gyro: " + convertToRealAngle(gyro.getAngle()));
        SmartDashboard.putNumber("Gyro", convertToRealAngle(gyro.getAngle()) );
    }

    public double getAngle() {
        return -(gyro.getAngle());
    }

    public double getConvertedRealAngle() {
        return convertToRealAngle(getAngle());
    }

    /**Turn robot to a passed angle using a proportional power P */
    public boolean turnToAngle(double angle, double P) {
        double error = angle - getAngle();
        System.out.println("error: " + error);
        rcw = error * P;
        System.out.println("rcw: " + rcw);
        drivetrain.turn(-rcw);
        System.out.println("angle from gyro: " + getAngle());
        System.out.println("angle raw: " + getAngle());
        return (getAngle() <= angle + 10 && getAngle() >= angle - 10);
    }

    /**Turn to angle using a proportional power P and integral value I */
    public boolean turnToAngle(double angle, double P, double I, double time) {
        double error = angle - getAngle();
        System.out.println("error: " + error);
        integral += I * (time / 60);
        rcw = error * P;
        System.out.println("rcw: " + rcw);
        drivetrain.turn(-rcw);
        System.out.println("angle from gyro: " + getAngle());
        System.out.println("angle raw: " + getAngle());
        return (getAngle() <= angle + 10 && getAngle() >= angle - 10);
    }

    public boolean DrivePID(double distance, double P) {
        if (distanceToGoalInches < minDistance) {
        double error = desiredDistance - threshold;
        } else if (distanceToGoalInches > maxDistance) {
        double error = desiredDistance + threshold;
        }
        go = error * P; 
        drivetrain.driveAuto(go);  
    }

    public boolean cancelTurn() {
        drivetrain.turn(0);
        integral = 0;
        return true;
    }
}

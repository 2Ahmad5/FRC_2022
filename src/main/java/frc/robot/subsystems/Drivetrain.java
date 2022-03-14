package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.util.Units;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.RobotMap.*;
import frc.robot.IO;

/**
 * The Drivetrain is an abstraction of the real-life system of two wheels powered by Falcon 500s that run our drive mechanism. It is an example of an FRC "Subsystem".
 */

public class Drivetrain implements Subsystem {
    //The Falcon 500s are a unit that include Talon FXs as their base motors, in which there is an encoder built in.
    WPI_TalonFX frontLeft = new WPI_TalonFX(ID_DRIVE_FL);
    WPI_TalonFX frontRight = new WPI_TalonFX(ID_DRIVE_FR);

    Timer timer = new Timer(); //for timing autonomous functions
    private DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight); //front motors are masters & control inputs for both front and back
    
    // Ahmad stuff start
    public void rightInvert(){
        frontRight.setInverted(true);
    }
    
    // Ahmad stuff end        

    /** drive function that can be called without having to pass in private vairables **/
    public void arcadeDrive() {
        if ((IO.getDriveTrigger(true) - IO.getReverseTrigger(true)) > 1 || (IO.getDriveTrigger(true) - IO.getReverseTrigger(true)) < -1) {
            System.out.println("out of bounds drive value. go to Drivetrain.java line 34 and edit to an in-bounds expression");
        } else {
            // drive.arcadeDrive(IO.getThrottle() * DRIVE_SPEED_MULT, IO.getLeftXAxis() * DRIVE_SPEED_MULT);
            if(IO.getThrottle() < 0){

                drive.arcadeDrive(IO.getThrottle(), IO.getLeftXAxis(true) * DRIVE_SPEED_MULT);
                IO.putNumberToSmartDashboard(("Velocity R: "), frontRight.getSelectedSensorVelocity());
                IO.putNumberToSmartDashboard(("Velocity L: "), frontLeft.getSelectedSensorVelocity());
                
                // System.out.println("Throttle: " + (Math.pow(IO.getThrottle(), 2) / 10));

            } else {
                drive.arcadeDrive(IO.getThrottle(), IO.getLeftXAxis(true) * DRIVE_SPEED_MULT);
            }
            IO.putNumberToSmartDashboard(("Right Drive Enc Value"),  frontLeft.getSelectedSensorPosition());
            IO.putNumberToSmartDashboard(("Left Drive Enc Value"),  frontRight.getSelectedSensorPosition());
         //   IO.putNumberToSmartDashboard(("Average Drive Enc Value"),  IO.getDriveDistance(frontRight.getSelectedSensorPosition(), frontLeft.getSelectedSensorPosition(), true));
        }
    }
    
    public void driveAuto(double speed) {
        drive.arcadeDrive(speed, 0);
    }


    /** drive a distance at a speed (uses encoder data) -- NEEDS UPDATING*/
    public void autoDistDrive(double dist, double speed) {
        double distance = distanceToNativeUnits(dist);
        double encoderAverage = (frontLeft.getSelectedSensorPosition() + frontRight.getSelectedSensorPosition()) / 2;
        if(distance > encoderAverage) {
            driveAuto(0.6);
        }
    }

    public void zeroEncoders() {
        frontLeft.setSelectedSensorPosition(0);
        frontRight.setSelectedSensorPosition(0);
    }




    /**initialize the drivetrain**/
    public void init() {
        /* Motor controllers default motor safety OFF.
            WPI drive trains default motor safety ON.
            Experiment with different enables below.... */
        //frontLeft.setSafetyEnabled(true);
        //frontRight.setSafetyEnabled(true);
        //drive.setSafetyEnabled(false);


        //Reset Motor Controllers to Factory Configuration
        frontLeft.configFactoryDefault();
        frontRight.configFactoryDefault();


        //Set Motor Direction and Encoder Sensor Phase
        frontLeft.setInverted(false);      // Positive is forward
        frontRight.setInverted(true);      // Invert so positive is forward

        frontLeft.setSensorPhase(false); // Check
        frontRight.setSensorPhase(true); // Check

        //init sensor position
        frontLeft.setSelectedSensorPosition(0);
        frontRight.setSelectedSensorPosition(0);

        //Set Brake/Coast Options
        frontLeft.setNeutralMode(BRAKE_MODE_DRIVE ? NeutralMode.Brake : NeutralMode.Coast);
        frontRight.setNeutralMode(BRAKE_MODE_DRIVE ? NeutralMode.Brake : NeutralMode.Coast);
        

        //Set Math.clamp Switch Positions
        final int kTimeoutMs = 30;  // Move to RobotMap??

        frontLeft.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, kTimeoutMs);
        frontRight.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, kTimeoutMs);
        
        /*
        * diff drive assumes (by default) that right side must be negative to move
        * forward. Change to 'false' so positive/green-LEDs moves robot forward
        */
        // drive.setRightSideInverted(false); // do not change this
        //1.13377687664 circ 8.667 ratio
    }

    public void initDefaultCommand() {}


    private int distanceToNativeUnits(double positionMeters){
        double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(DRIVE_RADIUS));
        double motorRotations = wheelRotations * DRIVE_GEAR_RATIO;
        int sensorCounts = (int)(motorRotations * COUNTS_PER_REV);
        return sensorCounts;
    }
    
    private double nativeUnitsToDistanceMeters(double sensorCounts){
        double motorRotations = (double)sensorCounts / COUNTS_PER_REV;
        double wheelRotations = motorRotations / DRIVE_GEAR_RATIO;
        double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(DRIVE_RADIUS));
        return positionMeters;
    }

    // public void driveDistance(double distance, double speed) {
    //     distance = distance / 12;
    //     int targetDistance = distanceToNativeUnits(distance);
    //     if()
    // }


    /**stops motors manually**/
    public void stopMotors() {
        frontLeft.stopMotor();
        frontRight.stopMotor();
    }
}


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;


//classes we make are imported here:
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Collector;
import frc.robot.autonomous.*;
import frc.robot.IO;

// Ahmad stuff start
ADXRS450_Gyro gyro = new ADXRS450_Gyro();
double kP = 0.05;
// Ahmad stuff end

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final Timer m_timer = new Timer();
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final Shooter shooter = new Shooter();
  public static final Collector collector = new Collector();
  public static final Collect collectCommand = new Collect(collector, shooter);

  //This is proactive - I'm not sure we'll end up NEEDING this, but I'm guessing it will be nescessary
  public static final double DRIVE_SENSITIVITY_MULT = 1;
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    // Ahmad stuff start
    drivetrain.rightInvert();
    // Ahmad stuff end
    
    
    //initiate subsystems 
    drivetrain.init();
    shooter.init();
    collector.init();
    
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
    collectCommand.setState(0);
    drivetrain.zeroEncoders();
    //repeat = -10;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
   // System.out.println("A");
     // drivetrain.autoDistDrive(2.16, 0.2);
   
    // Ahmad stuff start
    // Find the heading error; setpoint is 90
    double error = 90 - gyro.getAngle();

    // Turns the robot to face the desired direction
    drive.tankDrive(kP * error, -kP * error);
    // Ahmad stuff end
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    collectCommand.setState(1);
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // collector.runPneumatics();
    

    collectCommand.collect();

    drivetrain.arcadeDrive();
    // shooter.shooterControl();
    SmartDashboard.putBoolean("Ball Loaded?", collector.isBallLoaded());
    shooter.shooterPeriodic();
    //IO.putNumberToSmartDashboard("Lidar Distance", IO.getLidarDistance());
    //IO.putNumberToSmartDashboard("Vision Distance", IO.getVisionDistance());

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}


}

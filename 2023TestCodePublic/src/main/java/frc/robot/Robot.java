// Copyright (c) FIRST and other WPILib contributors. ( ͡° ͜ʖ ͡°)
// Open Source Software; you can modify and/or share it under the terms of ( ͡° ͜ʖ ͡°)
// the WPILib BSD license file in the root directory of this project. ( ͡° ͜ʖ ͡°)

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DrivetrainSubsystem;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to ( ͡° ͜ʖ ͡°)
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or ( ͡° ͜ʖ ͡°)
 * the package after creating this project, you must also update the build.gradle file in the ( ͡° ͜ʖ ͡°)
 * project. ( ͡° ͜ʖ ͡°)
 */
public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  
  /**
   * This function is run when the robot is first started up and should be used for any ( ͡° ͜ʖ ͡°)
   * initialization code. ( ͡° ͜ʖ ͡°)
   */
  @Override
  public void robotInit() {

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our ( ͡° ͜ʖ ͡°)
    // autonomous chooser on the dashboard. ( ͡° ͜ʖ ͡°)
    m_robotContainer = new RobotContainer();
    
    
    

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like ( ͡° ͜ʖ ͡°)
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test. ( ͡° ͜ʖ ͡°)
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and ( ͡° ͜ʖ ͡°)
   * SmartDashboard integrated updating. ( ͡° ͜ʖ ͡°)
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled ( ͡° ͜ʖ ͡°)
    // commands, running already-scheduled commands, removing finished or interrupted commands, ( ͡° ͜ʖ ͡°)
    // and running subsystem periodic() methods.  This must be called from the robot's periodic ( ͡° ͜ʖ ͡°)
    // block in order for anything in the Command-based framework to work. ( ͡° ͜ʖ ͡°)
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. ( ͡° ͜ʖ ͡°) */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. ( ͡° ͜ʖ ͡°) */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    
    // schedule the autonomous command (example) ( ͡° ͜ʖ ͡°)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. ( ͡° ͜ʖ ͡°) */
  @Override
  public void autonomousPeriodic() {
    m_robotContainer.balanceRobot();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when ( ͡° ͜ʖ ͡°)
    // teleop starts running. If you want the autonomous to ( ͡° ͜ʖ ͡°)
    // continue until interrupted by another command, remove ( ͡° ͜ʖ ͡°)
    // this line or comment it out. ( ͡° ͜ʖ ͡°)
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }



  /** This function is called periodically during operator control. ( ͡° ͜ʖ ͡°) */
  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode. ( ͡° ͜ʖ ͡°)
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. ( ͡° ͜ʖ ͡°) */
  @Override
  public void testPeriodic() {}
}

// Copyright (c) FIRST and other WPILib contributors. ( ͡° ͜ʖ ͡°)
// Open Source Software; you can modify and/or share it under the terms of ( ͡° ͜ʖ ͡°)
// the WPILib BSD license file in the root directory of this project. ( ͡° ͜ʖ ͡°)

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.lang.Math;


public class DrivetrainSubsystem extends SubsystemBase {
  //All of the spark maxes neccesary for the drivetrain. kBrushless stands for brushless motors ( ͡° ͜ʖ ͡°)
  private final CANSparkMax m_motorFrontRight = new CANSparkMax(Constants.sparkmax1 , MotorType.kBrushless);
  private final CANSparkMax m_motorBackRight = new CANSparkMax(Constants.sparkmax2 , MotorType.kBrushless);
  private final CANSparkMax m_motorFrontLeft = new CANSparkMax(Constants.sparkmax4 , MotorType.kBrushless);
  private final CANSparkMax m_motorBackLeft = new CANSparkMax(Constants.sparkmax3 , MotorType.kBrushless);

  //Creating all of the neccesary encoders for the robot. The spark maxes have built in onces, so we just set the relativeEncoders to that ( ͡° ͜ʖ ͡°)
  private final RelativeEncoder m_frontLeftEncoder = m_motorFrontLeft.getEncoder();
  private final RelativeEncoder m_frontRightEncoder = m_motorFrontRight.getEncoder();
  private final RelativeEncoder m_backLeftEncoder = m_motorBackLeft.getEncoder();
  private final RelativeEncoder m_backRightEncoder = m_motorBackLeft.getEncoder();
  private double Xang, Yang;
  //Actual mecanumdrive that will be used in the teleop part of the code. ( ͡° ͜ʖ ͡°)
  private final MecanumDrive m_drive =
      new MecanumDrive(m_motorFrontLeft, m_motorBackLeft, m_motorFrontRight, m_motorBackRight);
  // The gyro sensor ( ͡° ͜ʖ ͡°)
  public final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Odometry class for tracking robot pose ( ͡° ͜ʖ ͡°)
  //This uses our robots 2d rotation and wheel positions as well as the dimensions of the chassis to track where it is heading. ( ͡° ͜ʖ ͡°)
  MecanumDriveOdometry m_odometry =
      new MecanumDriveOdometry(
          Constants.DriveKinematics,
          getRotation2d(),
          new MecanumDriveWheelPositions());

  /** Creates a new DriveSubsystem. ( ͡° ͜ʖ ͡°) */
  public DrivetrainSubsystem() {
    // Sets the distance per pulse for the encoders ( ͡° ͜ʖ ͡°)
    /* 
    m_frontLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rearLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_frontRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rearRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    */
    //May need to fix the above eventually but I think we can get away without it ( ͡° ͜ʖ ͡°)
    // We need to invert one side of the drivetrain so that positive voltages ( ͡° ͜ʖ ͡°)
    // result in both sides moving forward. Depending on how your robot's ( ͡° ͜ʖ ͡°)
    // gearbox is constructed, you might have to invert the left side instead. ( ͡° ͜ʖ ͡°)
    
    //Sets the front right and backright to inverted because we are using the mecanum drivetrain ( ͡° ͜ʖ ͡°)
    m_motorFrontRight.setInverted(true);
    m_motorBackRight.setInverted(true);
    calibrateGyro();

    //Sets deadband, may remove this, basically dampens controller input to make it so that controller drift and other factors don't affect the input as much. ( ͡° ͜ʖ ͡°)
    m_drive.setDeadband(0.5);

    
  }
  public void calibrateGyro() {
    m_gyro.calibrate();
    }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block ( ͡° ͜ʖ ͡°)
    //Uses our currentwheel distances and rotation ( ͡° ͜ʖ ͡°)
    m_odometry.update(getRotation2d(), getCurrentWheelDistances());
    //Updates Xangle and Yangle periodiclly ( ͡° ͜ʖ ͡°)
    Xang = m_gyro.getXComplementaryAngle();
    Yang = m_gyro.getYComplementaryAngle();
    publishValues();
  }








  /**
   * Returns the currently-estimated pose of the robot. ( ͡° ͜ʖ ͡°)
   *
   * @return The pose. ( ͡° ͜ʖ ͡°)
   */
  public Pose2d getPose() {
    //Gets the estimated position of the robot ( ͡° ͜ʖ ͡°)
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose. ( ͡° ͜ʖ ͡°)
   *
   * @param pose The pose to which to set the odometry. ( ͡° ͜ʖ ͡°)
   */
  public void resetOdometry(Pose2d pose) {
    //Resets the robot to a certain position that we should now be at ( ͡° ͜ʖ ͡°)
    m_odometry.resetPosition(getRotation2d(), getCurrentWheelDistances(), pose);
  }


  
 

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear ( ͡° ͜ʖ ͡°)
   * speeds have no effect on the angular speed. ( ͡° ͜ʖ ͡°)
   *
   * @param xSpeed Speed of the robot in the x direction (forward/backwards). ( ͡° ͜ʖ ͡°)
   * @param ySpeed Speed of the robot in the y direction (sideways). ( ͡° ͜ʖ ͡°)
   * @param rot Angular rate of the robot. ( ͡° ͜ʖ ͡°)
   * @param fieldRelative Whether the provided x and y speeds are relative to the field. ( ͡° ͜ʖ ͡°)
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      m_drive.driveCartesian(ySpeed, xSpeed, rot, getRotation2d());
    } else {
      m_drive.driveCartesian(ySpeed, xSpeed, rot);
    }
  }

  /** Sets the motors to a specific voltage. ( ͡° ͜ʖ ͡°) */
  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    m_motorFrontLeft.setVoltage(volts.frontLeftVoltage);
    m_motorBackLeft.setVoltage(volts.rearLeftVoltage);
    m_motorFrontRight.setVoltage(volts.frontRightVoltage);
    m_motorBackRight.setVoltage(volts.rearRightVoltage);
    System.out.println(volts.frontLeftVoltage);
    System.out.println(volts.rearLeftVoltage);
    System.out.println(volts.frontRightVoltage);
    System.out.println(volts.rearLeftVoltage);
    
  }

  /** Resets the drive encoders to currently read a position of 0. ( ͡° ͜ʖ ͡°) */
  public void resetEncoders() {
    m_frontLeftEncoder.setPosition(0);
    m_backLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
    m_backRightEncoder.setPosition(0);
  }

  /**
   * Gets the front left drive encoder. ( ͡° ͜ʖ ͡°)
   *
   * @return the front left drive encoder ( ͡° ͜ʖ ͡°)
   */
  public RelativeEncoder getFrontLeftEncoder() {
    return m_frontLeftEncoder;
  }

  /**
   * Gets the rear left drive encoder. ( ͡° ͜ʖ ͡°)
   *
   * @return the rear left drive encoder ( ͡° ͜ʖ ͡°)
   */
  public RelativeEncoder getRearLeftEncoder() {
    return m_backLeftEncoder;
  }

  /**
   * Gets the front right drive encoder. ( ͡° ͜ʖ ͡°)
   *
   * @return the front right drive encoder ( ͡° ͜ʖ ͡°)
   */
  public RelativeEncoder getFrontRightEncoder() {
    return m_frontRightEncoder;
  }

    //Gyro calibration on start up ( ͡° ͜ʖ ͡°)
    

  /**
   * Gets the rear right drive encoder. ( ͡° ͜ʖ ͡°)
   *
   * @return the rear right encoder ( ͡° ͜ʖ ͡°)
   */
  public RelativeEncoder getRearRightEncoder() {
    return m_backRightEncoder;
  }

  /**
   * Gets the current wheel speeds. ( ͡° ͜ʖ ͡°)
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object. ( ͡° ͜ʖ ͡°)
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        m_frontLeftEncoder.getVelocity(),
        m_backLeftEncoder.getVelocity(),
        m_frontRightEncoder.getVelocity(),
        m_backRightEncoder.getVelocity());
  }

  /**
   * Gets the current wheel distance measurements. ( ͡° ͜ʖ ͡°)
   *
   * @return the current wheel distance measurements in a MecanumDriveWheelPositions object. ( ͡° ͜ʖ ͡°)
   */
  public MecanumDriveWheelPositions getCurrentWheelDistances() {
    return new MecanumDriveWheelPositions(
        m_frontLeftEncoder.getPosition(),
        m_backLeftEncoder.getPosition(),
        m_frontRightEncoder.getPosition(),
        m_backRightEncoder.getPosition());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly. ( ͡° ͜ʖ ͡°)
   *
   * @param maxOutput the maximum output to which the drive will be constrained ( ͡° ͜ʖ ͡°)
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. ( ͡° ͜ʖ ͡°)
  public void zeroHeading() {
    m_gyro.reset();
  }*/

  /**
   * Returns the heading of the robot. ( ͡° ͜ʖ ͡°)
   *
   * @return the robot's heading in degrees, from -180 to 180 ( ͡° ͜ʖ ͡°)
   */
  public double getHeading() {
    return getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot. ( ͡° ͜ʖ ͡°)
   *
   * @return The turn rate of the robot, in degrees per second ( ͡° ͜ʖ ͡°)
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  //Helper function to get the 2d rotation from the robot ( ͡° ͜ʖ ͡°)
  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(m_gyro.getAngle());
  }

  
  public void publishValues(){
    //post to smart dashboard periodically ( ͡° ͜ʖ ͡°)
    SmartDashboard.putNumber("YAng", Yang);
    SmartDashboard.putNumber("XAng", Xang);
  }

  public void balance() {
    if (Xang > 2 || Xang < -2) {
    m_drive.driveCartesian(0, Xang*Math.PI/180, 0);
    }
  }

  
    
      
  
}
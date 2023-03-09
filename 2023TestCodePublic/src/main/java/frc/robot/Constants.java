// Copyright (c) FIRST and other WPILib contributors. ( ͡° ͜ʖ ͡°)
// Open Source Software; you can modify and/or share it under the terms of ( ͡° ͜ʖ ͡°)
// the WPILib BSD license file in the root directory of this project. ( ͡° ͜ʖ ͡°)

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean ( ͡° ͜ʖ ͡°)
 * constants. This class should not be used for any other purpose. All constants should be declared ( ͡° ͜ʖ ͡°)
 * globally (i.e. public static). Do not put anything functional in this class. ( ͡° ͜ʖ ͡°)
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the ( ͡° ͜ʖ ͡°)
 * constants are needed, to reduce verbosity. ( ͡° ͜ʖ ͡°)
 */
public final class Constants {

    //Drivetrain Constants ( ͡° ͜ʖ ͡°)
    public static int sparkmax1 = 1;
    public static int sparkmax2 = 2;
    public static int sparkmax3 = 3;
    public static int sparkmax4 = 4;
    public static int gyroID = 5;

    public static int spark0 = 0; //lift ( ͡° ͜ʖ ͡°)
    public static int spark1 = 1;
    public static int spark2 = 2; //gripper ( ͡° ͜ʖ ͡°)
    public static int spark3 = 3; //gripper rotation ( ͡° ͜ʖ ͡°)

    public static int channel0 = 0;
    public static int channel1 = 1;

    //Joystick Constants ( ͡° ͜ʖ ͡°)
    public static int joystick = 0;

    //Lift Constants ( ͡° ͜ʖ ͡°)
    public static int SprocketDiameter = 1;
    public static int GearRatio = 1;

    public static final int EncoderCPR = 1024;
    public static final double WheelDiameterMeters = 0.2032;
    //Need to apply the gear ratio eventually since we are not mounted on the wheel shafts ( ͡° ͜ʖ ͡°)
    public static final double EncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts ( ͡° ͜ʖ ͡°)
        (WheelDiameterMeters * Math.PI) / (double) EncoderCPR;

    public static final double TrackWidth = 0.762;
    // Distance between centers of right and left wheels on robot ( ͡° ͜ʖ ͡°)
    public static final double WheelBase = 0.762;
    // Distance between centers of front and back wheels on robot ( ͡° ͜ʖ ͡°)
    //Kinematics ( ͡° ͜ʖ ͡°)
    
    


    public static final DifferentialDriveKinematics dDriveKinematics =
        new DifferentialDriveKinematics(WheelBase /2);

    public static final double dMaxSpeedMetersPerSecond = 1;
    public static final double dMaxAccelerationMetersPerSecondSquared = 1;
    public static final double dMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double dMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double dPXController = 40.201;
    public static final double dPYController = 40.201;
    public static final double dPThetaController = 40.201;

    // Constraint for the motion profilied robot angle controller ( ͡° ͜ʖ ͡°)
    public static final TrapezoidProfile.Constraints dThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            dMaxAngularSpeedRadiansPerSecond, dMaxAngularSpeedRadiansPerSecondSquared);

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT! ( ͡° ͜ʖ ͡°)
    // These characterization values MUST be determined either experimentally or theoretically ( ͡° ͜ʖ ͡°)
    // for *your* robot's drive. ( ͡° ͜ʖ ͡°)
    // The SysId tool provides a convenient method for obtaining these values for your robot. ( ͡° ͜ʖ ͡°)
    public static final SimpleMotorFeedforward dFeedforward = new SimpleMotorFeedforward(0.052645, 1.5796, 0.11143);

    // Example value only - as above, this must be tuned for your drive! ( ͡° ͜ʖ ͡°)
    public static final double PLeftVel = 0.15244;
    public static final double PRightVel = 0.15244;

    
    public static final MecanumDriveKinematics DriveKinematics =
        new MecanumDriveKinematics(
            new Translation2d(WheelBase / 2, TrackWidth / 2),
            new Translation2d(WheelBase / 2, -TrackWidth / 2),
            new Translation2d(-WheelBase / 2, TrackWidth / 2),
            new Translation2d(-WheelBase / 2, -TrackWidth / 2));

    public static final double MaxSpeedMetersPerSecond = 1;
    public static final double MaxAccelerationMetersPerSecondSquared = 1;
    public static final double MaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double MaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        
    public static final double PXController = 40.201;
    public static final double PYController = 40.201;
    public static final double PThetaController = 40.201;
        
        // Constraint for the motion profilied robot angle controller ( ͡° ͜ʖ ͡°)
    public static final TrapezoidProfile.Constraints ThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            MaxAngularSpeedRadiansPerSecond, MaxAngularSpeedRadiansPerSecondSquared);

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT! ( ͡° ͜ʖ ͡°)
    // These characterization values MUST be determined either experimentally or theoretically ( ͡° ͜ʖ ͡°)
    // for *your* robot's drive. ( ͡° ͜ʖ ͡°)
    // The SysId tool provides a convenient method for obtaining these values for your robot. ( ͡° ͜ʖ ͡°)
    public static final SimpleMotorFeedforward Feedforward = new SimpleMotorFeedforward(0.052645, 1.5796, 0.11143);

    // Example value only - as above, this must be tuned for your drive! ( ͡° ͜ʖ ͡°)
    public static final double PFrontLeftVel = 0.15244;
    public static final double PRearLeftVel = 0.15244;
    public static final double PFrontRightVel = 0.15244;
    public static final double PRearRightVel = 0.15244;

    

    
}

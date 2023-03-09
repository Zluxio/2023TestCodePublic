// Copyright (c) FIRST and other WPILib contributors. ( ͡° ͜ʖ ͡°)
// Open Source Software; you can modify and/or share it under the terms of ( ͡° ͜ʖ ͡°)
// the WPILib BSD license file in the root directory of this project. ( ͡° ͜ʖ ͡°)

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TurnOffCamMode;
import frc.robot.commands.TurnOnCamMode;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import edu.wpi.first.math.trajectory.constraint.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a ( ͡° ͜ʖ ͡°)
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot} ( ͡° ͜ʖ ͡°)
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including ( ͡° ͜ʖ ͡°)
 * subsystems, commands, and button mappings) should be declared here. ( ͡° ͜ʖ ͡°)
 */
//RobotContainer does all of the robot stuff basically. Holds all of the data members and main functions we need in terms of robot logic ( ͡° ͜ʖ ͡°)
public class RobotContainer {
  // m_controller for the xboxcontroller. This is why we use xbox controllers instead of ps4, there is an actual class for them versus the basic joystick class. ( ͡° ͜ʖ ͡°)
  XboxController m_controller = new XboxController(Constants.joystick);
  //Drivetrain Subsytem called m_robotDrive, this initializes our divetrainsubsytem which is in the subsytems folder. This is an instance of that class. ( ͡° ͜ʖ ͡°)
  private final DrivetrainSubsystem m_robotDrive = new DrivetrainSubsystem();
  //VisionSubsytem called m_vision, this is for the limelight and is an instance of the VisionSubsystem class. ( ͡° ͜ʖ ͡°)
  private final VisionSubsystem m_vision = new VisionSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final EndEffectorSubsystem m_gripper = new EndEffectorSubsystem();
  //m_field, this is used for the trajectory later on, it's okay to not know what this one does. It basically just creates a simulated field on the dashboard ( ͡° ͜ʖ ͡°)
  private Field2d m_field;
  //This is the SmartDashboard instance, we'll use this later on to display values and troubleshoot things instead of just the driver station. ( ͡° ͜ʖ ͡°)
  private SmartDashboard SmartDashboard;
  //Joystick buttons for the limelight ( ͡° ͜ʖ ͡°)
  private JoystickButton driver_X, driver_Y;




  /** The container for the robot. Contains subsystems, OI devices, and commands. ( ͡° ͜ʖ ͡°) */
  public RobotContainer() {
    // Configure the button bindings, this calls the configureButtonBindings further down ( ͡° ͜ʖ ͡°)
    configureButtonBindings();
    // Configure default commands ( ͡° ͜ʖ ͡°)
    // Set the default drive command to split-stick mecanum drive ( ͡° ͜ʖ ͡°)
    m_robotDrive.setDefaultCommand(


        // A split-stick mecanum command, with forward/backward controlled by the left ( ͡° ͜ʖ ͡°)
        // hand, and turning controlled by the right. ( ͡° ͜ʖ ͡°)
        //This may look really weird, however it's quite simple. RunCommand() takes an agrument which is just what it will run everytime ( ͡° ͜ʖ ͡°)
        // basically this creates a teleop command inline instead of us having to declare it in another file like the original 2023code does ( ͡° ͜ʖ ͡°)
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -m_controller.getLeftX(),
                    m_controller.getLeftY(),
                    -m_controller.getRightX(),
                    false),
            m_robotDrive));

    m_arm.setDefaultCommand(new RunCommand( () ->
      m_arm.setLiftMotor(
        m_controller.getRightTriggerAxis(), 
        m_controller.getLeftTriggerAxis()),
      m_arm));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by ( ͡° ͜ʖ ͡°)
   * instantiating a {@link GenericHID} or one of its subclasses ( ͡° ͜ʖ ͡°) ({@link 
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a ( ͡° ͜ʖ ͡°) {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}. ( ͡° ͜ʖ ͡°)
   */
  private void configureButtonBindings() {
    //This assigns the rightbumper to allow the driver to go between a maxoutput of .5 and 1 which will slow driving and allow for more accurate movement when needed. ( ͡° ͜ʖ ͡°)
    new JoystickButton(m_controller, Button.kRightBumper.value)
      .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
      .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));

    new JoystickButton(m_controller, Button.kA.value)
      .onFalse(new InstantCommand(() -> m_gripper.setGripperOutput(0)))
      .onTrue(new InstantCommand(() -> m_gripper.setGripperOutput(-0.5)));

    new JoystickButton(m_controller, Button.kB.value)
      .onFalse(new InstantCommand(() -> m_gripper.setGripperOutput(0)))
      .onTrue(new InstantCommand(() -> m_gripper.setGripperOutput(0.5)));
    
    new JoystickButton(m_controller, Button.kX.value)
      .onFalse(new InstantCommand(() -> m_gripper.setRotationOutput(0)))
      .onTrue(new InstantCommand(() -> m_gripper.setRotationOutput(-0.5)));

    new JoystickButton(m_controller, Button.kY.value)
      .onFalse(new InstantCommand(() -> m_gripper.setRotationOutput(0)))
      .onTrue(new InstantCommand(() -> m_gripper.setRotationOutput(0.5)));
    
    //This assigns the x and y button to turn the limelight cam mode on and off for the driver. Depending on if we use the limelight we might need to chagne the pipeline too ( ͡° ͜ʖ ͡°)
    // But witht he time we have left I'd rather get sysid and field trajectories working instead. ( ͡° ͜ʖ ͡°)
    driver_X = new JoystickButton(m_controller, XboxController.Button.kX.value);
    driver_X.onTrue(new TurnOnCamMode(m_vision));
    driver_Y = new JoystickButton(m_controller, XboxController.Button.kY.value);
    driver_Y.onTrue(new TurnOffCamMode(m_vision));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class. ( ͡° ͜ʖ ͡°)
   *
   * @return the command to run in autonomous ( ͡° ͜ʖ ͡°)
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory ( ͡° ͜ʖ ͡°)
    // This basically gives the trajectory creator configuration parameters that it will use to then generate a trajectory ( ͡° ͜ʖ ͡°)
    DifferentialDriveVoltageConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            Constants.dFeedforward, Constants.dDriveKinematics, 1.2);


    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.MaxSpeedMetersPerSecond,
                Constants.MaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed ( ͡° ͜ʖ ͡°)
            .setKinematics(Constants.DriveKinematics)
            .addConstraint(autoVoltageConstraint);
            
          

    // An example trajectory to follow.  All units in meters. ( ͡° ͜ʖ ͡°)
    // The trajectory will start 1 meter up, then one meter to the right, then again to the right to travel a total of 3 meters forward. ( ͡° ͜ʖ ͡°)
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of( new Translation2d(1, 0), new Translation2d(2, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);
    Trajectory gyroRoute =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(90)),
            List.of(new Translation2d(0, 0.6096), new Translation2d(1.016, .6096), new Translation2d(1.016, 1.3208)),
            new Pose2d(0, 1.3208, new Rotation2d(0)),
            config);
    Trajectory cubeRoute =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(180)),
            List.of(new Translation2d(-0.1, 0)),
            new Pose2d(4.9276, 0, new Rotation2d(0)),
            config);
    m_robotDrive.calibrateGyro();

    //This shows the field on the smartdashboard ( ͡° ͜ʖ ͡°)
    m_field = new Field2d();
    //This sends the smartdashboard the neccesary data ( ͡° ͜ʖ ͡°)
    SmartDashboard.putData(m_field);
    //This just gets the trajectory object so it can show ( ͡° ͜ʖ ͡°)
    m_field.getObject("traj").setTrajectory(exampleTrajectory);
    //This is where it gets complicated. To better understand this I would recommend reading the docs. It's already documented there and I can't really explain it in a comment. ( ͡° ͜ʖ ͡°)
    //They have visuals and explanations of how this works. It basically just performs the trajectory by taking in a bunch of pidcontrollers and values as well as the position ( ͡° ͜ʖ ͡°)
    //of the robot and the odometry of the robot and updates it in a self-contained loop as it moves. ( ͡° ͜ʖ ͡°)
    MecanumControllerCommand mecanumControllerCommand =
        new MecanumControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            Constants.Feedforward,
            Constants.DriveKinematics,

            // Position contollers ( ͡° ͜ʖ ͡°)
            new PIDController(Constants.PXController, 0, 0),
            new PIDController(Constants.PYController, 0, 0),
            new ProfiledPIDController(
                Constants.PThetaController, 0, 0, Constants.ThetaControllerConstraints),

            // Needed for normalizing wheel speeds ( ͡° ͜ʖ ͡°)
            Constants.MaxSpeedMetersPerSecond,

            // Velocity PID's ( ͡° ͜ʖ ͡°)
            new PIDController(Constants.PFrontLeftVel, 0, 0),
            new PIDController(Constants.PRearLeftVel, 0, 0),
            new PIDController(Constants.PFrontRightVel, 0, 0),
            new PIDController(Constants.PRearRightVel, 0, 0),
            m_robotDrive::getCurrentWheelSpeeds,
            m_robotDrive::setDriveMotorControllersVolts, // Consumer for the output motor voltages ( ͡° ͜ʖ ͡°)
            m_robotDrive);
            

    // Reset odometry to the starting pose of the trajectory. ( ͡° ͜ʖ ͡°)
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end. ( ͡° ͜ʖ ͡°)
    return mecanumControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
  

  public void balanceRobot() {
    m_robotDrive.balance();
  }
  
}
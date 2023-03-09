package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.VisionSubsystem;

public class TurnOnCamMode extends InstantCommand {
    private final VisionSubsystem m_vision;

    public TurnOnCamMode(VisionSubsystem vision) {
        m_vision = vision;
        // Use addRequirements() here to declare subsystem dependencies. ( ͡° ͜ʖ ͡°)
        addRequirements(m_vision);
        
    }

    // Called when the command is initially scheduled. ( ͡° ͜ʖ ͡°)
    @Override
    public void execute() {
        m_vision.setCamMode(0);
        m_vision.setLedMode(3);
        System.out.println("hello");
    }
}

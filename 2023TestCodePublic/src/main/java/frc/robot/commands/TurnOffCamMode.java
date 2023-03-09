package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.VisionSubsystem;

public class TurnOffCamMode extends InstantCommand {
    private final VisionSubsystem m_vision;

    public TurnOffCamMode(VisionSubsystem vision) {
        m_vision = vision;
        // Use addRequirements() here to declare subsystem dependencies. ( ͡° ͜ʖ ͡°)
        addRequirements(m_vision);
        
    
    }

    @Override
    public void execute() {
        m_vision.setCamMode(1);
        m_vision.setLedMode(1);
        
        System.out.println("hello");
    }
}

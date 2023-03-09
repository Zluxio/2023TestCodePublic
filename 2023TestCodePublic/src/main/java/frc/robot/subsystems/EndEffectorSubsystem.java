package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;


public class EndEffectorSubsystem extends SubsystemBase {
    private Spark m_GripperMotor;
    private Spark m_RotationMotor;
    private DigitalInput leftLimitSwitch;
    private DigitalInput rightLimitSwitch;


    public EndEffectorSubsystem() {
        m_GripperMotor = new Spark(Constants.spark0);
        m_RotationMotor = new Spark(Constants.spark1);
        leftLimitSwitch = new DigitalInput(0);
        rightLimitSwitch = new DigitalInput(1);
    }

    public void setGripperOutput(double output) { 
        if (leftLimitSwitch.get()) {
            m_GripperMotor.set(0);
        } else if (rightLimitSwitch.get()) {
            m_GripperMotor.set(0);
        } else {
            m_GripperMotor.set(output);
        }
                                                                                      
      }

    public void setRotationOutput (double output) {
        m_RotationMotor.set(output);
    }
    
}

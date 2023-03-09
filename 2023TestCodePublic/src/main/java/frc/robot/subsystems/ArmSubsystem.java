package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//IGNORE THIS, LIFT TEAM SUX AND DIDN'T DO ENCODERS SO WE CAN'T DO THIS NOW XD ( ͡° ͜ʖ ͡°)
public class ArmSubsystem extends SubsystemBase {
    //floor, middle, high settings for both cone and cube ( ͡° ͜ʖ ͡°)
    //cubeFloor = 4.75 inch (low enough for middle of cube) ( ͡° ͜ʖ ͡°)
    //cubeMiddle = 23.5 inches ( ͡° ͜ʖ ͡°)
    //cubeHigh = 35.5 inches ( ͡° ͜ʖ ͡°)
    //coneFloor = 4.75 inch (low enough to have enough to grip, can move this to the floor if needed but I think grabbing the middle is a better idea) ( ͡° ͜ʖ ͡°)
    //coneMiddle = 35 inches (not exact value to get it above it basically and just drop it) ( ͡° ͜ʖ ͡°)
    //coneHigh = 47 inches (not exact value to get it above it basically and just drop it) ( ͡° ͜ʖ ͡°)
    
    //Assume implementation with Neo motors and Spark Max ( ͡° ͜ʖ ͡°)
    private Spark m_liftMotor1;
    private Spark m_liftMotor2;
    
    //6 ft max ( ͡° ͜ʖ ͡°)
    //Want to have a minimum of around 4.75 inch ( ͡° ͜ʖ ͡°)

    XboxController m_controller = new XboxController(Constants.joystick);
    
    public ArmSubsystem(){
        m_liftMotor1 = new Spark(Constants.spark2);
        m_liftMotor2 = new Spark(Constants.spark3);
        

    }

    public void setLiftMotor(double leftTrigger, double rightTrigger) {
        //leftTrigger goes down
        if(leftTrigger > 0.1){
            m_liftMotor1.set(-leftTrigger * 1);
            m_liftMotor2.set(-leftTrigger * 1);
        }
        //rightTrigger goes up
        else if (rightTrigger > 0.1){
            m_liftMotor1.set(rightTrigger * 1);
            m_liftMotor2.set(rightTrigger * 1);
        }
        else{
            m_liftMotor1.set(0);
            m_liftMotor2.set(0);
        }
    }

}

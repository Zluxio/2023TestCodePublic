package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class VisionSubsystem extends SubsystemBase {
    //All the data members for the limelight that is neccesary to get it to connect to the robot and the network tables. ( ͡° ͜ʖ ͡°)
    private NetworkTable m_LimelightTable;
    private double tx, ty, ta, tv;
    private ArrayList<Double> m_TargetList;
    private int m_MaxEntries = 50;
    private NetworkTableEntry m_LedEntry;
    private NetworkTableEntry m_CamEntry;
    //Initializes our VisionSubsystem in the constructor ( ͡° ͜ʖ ͡°)
    public VisionSubsystem(){
        m_LimelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        m_TargetList = new ArrayList<Double>(m_MaxEntries);
        m_CamEntry = m_LimelightTable.getEntry("camMode");
        m_LedEntry = m_LimelightTable.getEntry("ledMode");

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run ( ͡° ͜ʖ ͡°)
        tv = m_LimelightTable.getEntry("tv").getDouble(0);
        tx = m_LimelightTable.getEntry("tx").getDouble(0);
        ta = m_LimelightTable.getEntry("ta").getDouble(0);

        if (m_TargetList.size() >= m_MaxEntries) {
            m_TargetList.remove(0);
          }
        m_TargetList.add(ta);
        
        publishValues();
    }

    public double getTX() {
        return tx;
    }

    public double getTV() {
        return tv;
    }
    
    public double getTA() {
        double sum = 0;
    
        for (Double num : m_TargetList) { 		      
          sum += num.doubleValue();
        }

        return sum/m_TargetList.size();
    }

    public void setCamMode(int mode){
        m_CamEntry.setDouble((mode));
    }
    public void setLedMode(int mode){
        m_LedEntry.setDouble((mode));
    }

    public void publishValues(){
        //post to smart dashboard periodically ( ͡° ͜ʖ ͡°)
        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightY", tv);
        SmartDashboard.putNumber("LimelightArea", ta);

    }
}

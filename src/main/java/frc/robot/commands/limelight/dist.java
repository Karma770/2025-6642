package frc.robot.commands.limelight;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;

public class dist {
    private final ShuffleboardTab tab = Shuffleboard.getTab("Limelight");
    private final GenericEntry limeEntry = tab.add("position", 0).getEntry();
    private final String limelightName = "limelight";






         public void periodic() {
        double tA = LimelightHelpers.getTA(limelightName); // ✅ Get AprilTag area
        double knownDistance = 37.0;  
        double knownArea = 2.9; 

        double estimatedDistance=(knownArea * knownDistance) / Math.max(tA, 0.1); 
        ;
                // Update SmartDashboard every cycle with the current
                SmartDashboard.putNumber("Position", estimatedDistance);
        limeEntry.setDouble(estimatedDistance);

    }
    
}

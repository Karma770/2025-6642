package frc.robot.commands.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LimelightLateralAlignCommandY extends Command { 

    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController pidY = new PIDController(0.002, 0, 0.0001);  // Forward/backward alignment
    private final PIDController pidX = new PIDController(0.02, 0, 0.001);   // Lateral alignment
    private final PIDController pidRotate = new PIDController(0.01, 0, 0.0005); // Rotation alignment
    private final String limelightName = "limelight";
    private final double desiredOffsetInchesY;
    private final double desiredOffsetInchesX;
    private final double conversionFactor = 1.0; // Inches per degree
    private final double rotationTolerance = 2.0; // Degrees

    private double previousDistance = 36.0; 

    public LimelightLateralAlignCommandY(CommandSwerveDrivetrain drivetrain, double desiredOffsetInchesY, double desiredOffsetInchesX) {
        this.drivetrain = drivetrain;
        this.desiredOffsetInchesY = desiredOffsetInchesY;
        this.desiredOffsetInchesX = desiredOffsetInchesX;

        addRequirements(drivetrain);
        
        pidY.setSetpoint(0.0);
        pidY.setTolerance(1); 

        pidX.setSetpoint(0.0); 
        pidX.setTolerance(0.5); 

        pidRotate.setSetpoint(0.0); // We want to be parallel to the tag
        pidRotate.setTolerance(rotationTolerance);
    }

    @Override
    public void execute() {
        boolean hasTarget = LimelightHelpers.getTV(limelightName);
        double tA = LimelightHelpers.getTA(limelightName); 
        double tx = LimelightHelpers.getTX(limelightName); // Horizontal offset
        double ts = LimelightHelpers.getTS(limelightName); // Skew (rotation)

        if (!hasTarget) {
            drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
            return;
        }
        
        // Estimate distance using area
        double knownDistance = 37.0;  
        double knownArea = 2.9;       
        double estimatedDistance = (knownArea * knownDistance) / Math.max(tA, 0.1); 
        double estimatedLatx = tx * conversionFactor;
        double lateralError = estimatedLatx - desiredOffsetInchesX;
        double estimatedAngle = ts *1;
        double angleError = estimatedAngle - 0;

        // Apply Exponential Smoothing Filter
        double smoothingFactor = 0.8;
        estimatedDistance = (smoothingFactor * previousDistance) + ((1 - smoothingFactor) * estimatedDistance);
        previousDistance = estimatedDistance;
        
        double distanceError = estimatedDistance - desiredOffsetInchesY;
        
        // Deadband for forward/backward movement
        double forwardSpeedCommand;
        if (Math.abs(distanceError) > 10) {  
            forwardSpeedCommand = 0.7 * Math.signum(distanceError);  
        } else {
            forwardSpeedCommand = -pidY.calculate(distanceError);
        }
        
        // Clamp speed for safety
        forwardSpeedCommand = Math.max(-0.7, Math.min(0.7, forwardSpeedCommand)); 
    
        // Lateral movement correction
        double lateralSpeedCommand = pidX.calculate(lateralError); 
        lateralSpeedCommand = Math.max(-0.5, Math.min(0.5, lateralSpeedCommand)); 
    
        // **Rotation alignment to stay parallel**
        double rotationSpeedCommand = pidRotate.calculate(angleError); 
        rotationSpeedCommand = Math.max(-0.5, Math.min(0.5, rotationSpeedCommand)); // Clamp for safety
    
        // Apply forward, lateral, and rotational movement
        drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds.withSpeeds(
            new ChassisSpeeds(forwardSpeedCommand, lateralSpeedCommand, rotationSpeedCommand)
        ));
    }
    
    @Override
    public boolean isFinished() {
        return pidY.atSetpoint() && pidX.atSetpoint() && pidRotate.atSetpoint();
    }
    
    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
        pidY.reset();
        pidX.reset();
        pidRotate.reset();
    }
}

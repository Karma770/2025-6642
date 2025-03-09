package frc.robot.commands.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LimelightLateralAlignCommandY extends Command { 
    
    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController pidController = new PIDController(.05, 0.0, .01);
    private final String limelightName = "limelight";
    private final double desiredOffsetInches;

    private double previousDistance = 36.0; // ✅ ADDED for smoothing distance updates

    public LimelightLateralAlignCommandY(CommandSwerveDrivetrain drivetrain, double desiredOffsetInches) {
        this.drivetrain = drivetrain;
        this.desiredOffsetInches = desiredOffsetInches;
        addRequirements(drivetrain);
        pidController.setSetpoint(0.0);
        pidController.setTolerance(1); 
    }

    @Override
    public void execute() {
        boolean hasTarget = LimelightHelpers.getTV(limelightName);
        double tA = LimelightHelpers.getTA(limelightName); // ✅ Get AprilTag area

        if (!hasTarget) {
            drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
            return;
        }

        // ✅ Estimate distance using tA
        double knownDistance = 37.0;  
        double knownArea = 2.9;       
        double estimatedDistance = (knownArea * knownDistance) / Math.max(tA, 0.1); 

        // ✅ Apply smoothing filter (Rolling Average)
        double smoothingFactor = 0.8;  
        estimatedDistance = (smoothingFactor * previousDistance) + ((1 - smoothingFactor) * estimatedDistance);
        previousDistance = estimatedDistance; // Store for next loop

        double distanceError = estimatedDistance - desiredOffsetInches;

        // ✅ Deadband: Stop movement if within 2 inches
        if (Math.abs(distanceError) < 2) {
            drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
            return;
        }

        // ✅ Calculate forward/backward movement using PID
        double forwardSpeedCommand = pidController.calculate(distanceError);

        // ✅ Limit max speed to prevent overshooting
        forwardSpeedCommand = Math.max(0.5, Math.min(0.5, forwardSpeedCommand));

        // ✅ Apply movement (forward/backward based on Y-axis)
        ChassisSpeeds speeds = new ChassisSpeeds(forwardSpeedCommand, 0.0, 0.0);
        drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds.withSpeeds(speeds));
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds()));
    }
}
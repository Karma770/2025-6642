package frc.robot.commands.limelight;


import edu.wpi.first.math.controller.PIDController; 
import edu.wpi.first.math.kinematics.ChassisSpeeds;  
import frc.robot.LimelightHelpers; 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LimelightLateralAlignCommandX extends Command{ 
    
    
private final CommandSwerveDrivetrain drivetrain;

// PID controller to drive lateral error (in inches) to zero. 
private final PIDController pidController = new PIDController(0.03, 0.0, 0.02);
private double previousCommand = 0.0;
private final double smoothingFactor = 0.8; // Higher = smoother, Lower = faster response

 // Name of your limelight 
 private final String limelightName = "limelight";
// Conversion factor to convert tx (degrees) to lateral offset (inches).
// For a more accurate conversion, you could use distance estimates:
// lateralOffset = distance * Math.tan(Math.toRadians(tx));
// Here, we'll use a constant factor that you should calibrate.
private final double conversionFactor = 1.0; // inches per degree

// If you want the robot to shift a fixed amount relative to the target,
// you can set a desired lateral offset. For example, if you want the robot
// to be exactly centered, set desiredOffset = 0.
// If you want it to be 6 inches to one side, set desiredOffset = 6 (or -6 for the opposite side).
private final double desiredOffsetInches;

public LimelightLateralAlignCommandX(CommandSwerveDrivetrain drivetrain, double desiredOffsetInches) {
    this.drivetrain = drivetrain;
    this.desiredOffsetInches = desiredOffsetInches;
    addRequirements(drivetrain);
    // The PID controller will try to drive the lateral error (calculated below) to 0.
    pidController.setSetpoint(0.0);
    pidController.setTolerance(1); // Adjust tolerance as needed (in inches)
}
@Override
public void execute() {
    double tx = LimelightHelpers.getTX(limelightName);
    double currentLateralOffset = tx * conversionFactor;
    double error = currentLateralOffset - desiredOffsetInches;
    
    // ✅ Increase deadband to prevent jitter
    if (Math.abs(error) < 1.5) { 
        drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
        return; 
    }
    
    // ✅ Compute lateral speed command with PID
    double lateralSpeedCommand = pidController.calculate(error);

    // ✅ Apply smoothing to reduce jitter
    lateralSpeedCommand = (smoothingFactor * previousCommand) + ((1 - smoothingFactor) * lateralSpeedCommand);
    previousCommand = lateralSpeedCommand;

    // ✅ Limit speed to prevent overshooting
    lateralSpeedCommand = Math.max(-0.3, Math.min(0.3, lateralSpeedCommand));

    // ✅ Send speeds to drivetrain
    drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0.0, lateralSpeedCommand, 0.0)));
}

@Override
public boolean isFinished() {
    // End the command when the error is within the tolerance.
    return pidController.atSetpoint();
}

@Override
public void end(boolean interrupted) {
    drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
    pidController.reset();  // ✅ Reset PID for next use
}

}
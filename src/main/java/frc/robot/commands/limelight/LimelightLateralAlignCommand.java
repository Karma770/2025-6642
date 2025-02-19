package frc.robot.commands.limelight;


import edu.wpi.first.math.controller.PIDController; 
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers; 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LimelightLateralAlignCommand extends Command{ 
    
    
private final CommandSwerveDrivetrain drivetrain;

// PID controller to drive lateral error (in inches) to zero. 
private final PIDController pidController = new PIDController(.05, 0.0, .01);
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

public LimelightLateralAlignCommand(CommandSwerveDrivetrain drivetrain, double desiredOffsetInches) {
    this.drivetrain = drivetrain;
    this.desiredOffsetInches = desiredOffsetInches;
    addRequirements(drivetrain);
    // The PID controller will try to drive the lateral error (calculated below) to 0.
    pidController.setSetpoint(0.0);
    pidController.setTolerance(1); // Adjust tolerance as needed (in inches)
}

@Override
public void execute() {
    // Read the horizontal offset (tx) in degrees from the Limelight.
    double tx = LimelightHelpers.getTX(limelightName);
    
    // Convert the tx value to a lateral offset in inches.
    // For a rough conversion, multiply tx by a calibrated conversion factor.
    double currentLateralOffset = tx * conversionFactor;
    
    // Calculate the error relative to the desired offset.
    // For example, if you want the robot centered (0 inches), the error is currentLateralOffset - 0.
    // If you want a 6-inch offset, the error is currentLateralOffset - 6.
    double error = currentLateralOffset - desiredOffsetInches;


    SmartDashboard.putNumber("error",error);


    // Get the lateral adjustment command from the PID controller.
    double lateralSpeedCommand = pidController.calculate(error);
    
    // Create a chassis speeds object:
    // Assuming that x is forward and y is lateral (positive y may be left depending on your configuration).
    // Here we only drive laterally (set forward speed to 0 and no rotation).
    ChassisSpeeds speeds = new ChassisSpeeds(0.0, lateralSpeedCommand, 0.0);
    
    // Command the drivetrain.
    drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds.withSpeeds(speeds)
    );
}

@Override
public boolean isFinished() {
    // End the command when the error is within the tolerance.
    return pidController.atSetpoint();
}

@Override
public void end(boolean interrupted) {
    // Stop the drivetrain when the command ends.
    drivetrain.setControl(
        drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds())
    );
}}
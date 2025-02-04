package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.controller.PIDController;

class PIDControllerConfigurable extends PIDController {
  public PIDControllerConfigurable(double kP, double kI, double kD) {
      super(kP, kI, kD);
  }
  
  public PIDControllerConfigurable(double kP, double kI, double kD, double tolerance) {
      super(kP, kI, kD);
      this.setTolerance(tolerance);
  }
}
public class AlignCommand extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final VisionSubsystem m_Limelight;
  private final int tagID;

  private static final PIDControllerConfigurable rotationalPidController = new PIDControllerConfigurable(0.075, 0, 0, 0.01);
  private static final PIDControllerConfigurable xPidController = new PIDControllerConfigurable(0.3, 0.01, 0.01, 0.01);
  private static final PIDControllerConfigurable yPidController = new PIDControllerConfigurable(0.3, 0, 0, 0.3);
  private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  public AlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight) {
    this.m_drivetrain = drivetrain; 
    this.m_Limelight = limelight;
    this.tagID=m_Limelight.getClosestFiducial().id;
    addRequirements(m_Limelight);
  }

  public AlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, int tagID) {
    this.m_drivetrain = drivetrain;
    this.m_Limelight = limelight;
    this.tagID = tagID;
    addRequirements(m_Limelight);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    
    RawFiducial fiducial;

    try {
      fiducial = m_Limelight.getFiducialWithId(this.tagID);
      System.out.println(fiducial.distToRobot);

      final double rotationalRate = rotationalPidController.calculate(2*fiducial.txnc, 0.0) * 0.75* 0.9;
      
      final double velocityX = xPidController.calculate(fiducial.distToRobot, 0.1) * 4.73 * 0.7;
        
      if (rotationalPidController.atSetpoint() && xPidController.atSetpoint() && yPidController.atSetpoint()) {
        this.end(true);
      }

      SmartDashboard.putNumber("txnc", fiducial.txnc);
      SmartDashboard.putNumber("distToRobot", fiducial.distToRobot);
      SmartDashboard.putNumber("rotationalPidController", rotationalRate);
      SmartDashboard.putNumber("xPidController", velocityX);
      m_drivetrain.setControl(
          alignRequest.withRotationalRate(-rotationalRate).withVelocityX(-velocityX));
 
    } catch (VisionSubsystem.NoSuchTargetException nste) {
    }
  }

  @Override
  public boolean isFinished() {
    return rotationalPidController.atSetpoint() && xPidController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.applyRequest(() -> idleRequest);
    
  }
}
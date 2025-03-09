package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.climber;

public class Climb extends Command {
  private  climber arm;
  private double setpoint;
  /** Creates a new RunArmClosedLoop. */
  public Climb(climber ar, double target) {
    arm = ar;
    setpoint = target;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    arm.runTovel(setpoint);
    System.out.println("Climber RUNNING");
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   arm.hold();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class HoldArm extends Command {
  Elevator arm;
  /** Creates a new HoldArm. */
  public HoldArm(Elevator ar) {
    arm = ar;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.hold();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class RunArmOpenLoop extends Command {
  private Elevator arm;
  private double supplier;
  private boolean override;
  /** Creates a new RunArmOpenLoop. */
  public RunArmOpenLoop(Elevator ar, double input, boolean isOverride) {
    supplier = input;
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
    if(override) {
      isFinished();
    }
    else {
      arm.runOpenLoop(supplier);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //arm.stopArm(0);
    arm.hold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
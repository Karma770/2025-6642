// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeA;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakePivotA;



public class RunIntakeOpenLoop extends Command {

  private IntakePivotA intake;
  private double pos;
  /** Creates a new RunIntakeOpenLoop. */
  public RunIntakeOpenLoop(IntakePivotA m_IntakePivotA, double input) {
    intake = m_IntakePivotA;
    pos = input;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakePivotA);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runToPosition(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

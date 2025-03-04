// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake;

public class HoldIntake extends Command {
  private intake intake;
  private double setpoint;
  /** Creates a new HoldIntake. */
  public HoldIntake(intake in) {
    intake = in;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(in);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint = intake.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.hold(setpoint);
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

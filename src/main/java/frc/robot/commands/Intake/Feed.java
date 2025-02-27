// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.*;
import frc.robot.subsystems.intake;

public class Feed extends Command {
  private intake intake;
  private double startTime;
  private double timeout;
  /** Creates a new Feed. */
  public Feed(intake in) {
    intake = in;
    startTime = 0.0;
    timeout = 1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(in);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runAtVelocity(SystemConfig.CORAL_INTAKE_SPEED);
   // intake.runOpenLoop(IntakeConstants.kReverseSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

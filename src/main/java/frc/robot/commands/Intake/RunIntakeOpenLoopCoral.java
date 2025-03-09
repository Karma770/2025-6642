// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.intake;



public class RunIntakeOpenLoopCoral extends Command {

  private IntakePivot intake;
  private double speed;
  /** Creates a new RunIntakeOpenLoop. */
  public RunIntakeOpenLoopCoral(IntakePivot m_IntakePivot, double input) {
    intake = m_IntakePivot;
    speed = input;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakePivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     intake.runToPosition(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
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

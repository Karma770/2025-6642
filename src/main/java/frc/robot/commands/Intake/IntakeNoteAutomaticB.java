// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.*;
//import frc.robot.Commands.LEDs.SetLED;
//import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.intake;

public class IntakeNoteAutomaticB extends Command {
  //private final BlinkinSubsystem m_blink = new BlinkinSubsystem(0);
//private final Blinkin m_blink = new Blinkin(0);
  private intake intake;
  private boolean end;
  /** Creates a new IntakeNoteAutomatic. */
  public IntakeNoteAutomaticB(intake in) {
    intake = in;
    end = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(in);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!intake.isCoral()) {
      //intake.autoIntake();
      intake.runOpenLoop(SystemConfig.CORAL_INTAKE_SPEED);
      System.out.println("INTAKE RUNNING");
      //new SetLED(m_blink, -0.99);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //new SetLED(m_blink, -0.99);
  intake.stopIntake(0);

  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      //System.out.println("Intake Stoped");
      return intake.isCoral();
 
    
     
  }
}

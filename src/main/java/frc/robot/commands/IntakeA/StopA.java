// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeA;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.*;
//import frc.robot.Commands.LEDs.SetLED;
//import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.intakeA;

public class StopA extends Command {
  //private final BlinkinSubsystem m_blink = new BlinkinSubsystem(0);
//private final Blinkin m_blink = new Blinkin(0);
  private intakeA intake;
  private boolean end;


  /** Creates a new IntakeNoteAutomatic. */
  public StopA(intakeA in) {
    intake = in;
    
    end = false;




    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(in);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    intake.stopIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
  

    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {


  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return true; 
 
    
     
  }
}

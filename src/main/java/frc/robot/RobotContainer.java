// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.CANConfig;
import frc.robot.commands.Arm.RunArmClosedLoop;
import frc.robot.commands.Intake.IntakeSUCK;
import frc.robot.commands.Intake.Intakespit;
import frc.robot.commands.Intake.RunIntakeOpenLoopCoral;
import frc.robot.commands.Intake.IntakeSUCK;
import frc.robot.commands.IntakeA.DropA;
import frc.robot.commands.IntakeA.IntakeNoteAutomatic;
import frc.robot.commands.IntakeA.RunIntakeOpenLoop;
import frc.robot.commands.IntakeA.StopA;
import frc.robot.commands.limelight.LimelightLateralAlignCommandY;
import frc.robot.commands.limelight.LimelightLateralAlignCommandX;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakePivotA;
import frc.robot.subsystems.intake;
//import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.intakeA;





public class RobotContainer {
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


    private final Elevator m_Elevator = new Elevator( CANConfig.ELEVATOR_FRONT, CANConfig.ELEVATOR_BACK);
    private final intakeA m_intakeA = new intakeA();
    private final IntakePivotA m_IntakePivotA = new IntakePivotA();
    final IntakePivot m_IntakePivot = new IntakePivot();
    private final intake m_intake = new intake(0);








    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController gamepad = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    


    SequentialCommandGroup L1= new SequentialCommandGroup(
        new RunArmClosedLoop(m_Elevator, ArmConstants.L1pos),
        new RunIntakeOpenLoopCoral(m_IntakePivot, ArmConstants.CPivotScore),
        new Intakespit(m_intake).withTimeout(.5),
        new RunIntakeOpenLoopCoral(m_IntakePivot,ArmConstants.CPivotScore),
        new RunArmClosedLoop(m_Elevator, ArmConstants.L1pos)
      );
    SequentialCommandGroup L2= new SequentialCommandGroup(
        new RunArmClosedLoop(m_Elevator, ArmConstants.L2pos),
        new RunIntakeOpenLoopCoral(m_IntakePivot, ArmConstants.CPivotScore),
        new Intakespit(m_intake).withTimeout(.5),
        new RunIntakeOpenLoopCoral(m_IntakePivot,ArmConstants.CPivotScore),
        new RunArmClosedLoop(m_Elevator, ArmConstants.L1pos)
      );
    SequentialCommandGroup L3= new SequentialCommandGroup(
        new RunArmClosedLoop(m_Elevator, ArmConstants.L3pos),
        new RunIntakeOpenLoopCoral(m_IntakePivot, ArmConstants.CPivotScore),
        new Intakespit(m_intake).withTimeout(.5),
        new RunIntakeOpenLoopCoral(m_IntakePivot,ArmConstants.CPivotScore),
        new RunArmClosedLoop(m_Elevator, ArmConstants.L1pos)
      );
    SequentialCommandGroup L4= new SequentialCommandGroup(
        new RunArmClosedLoop(m_Elevator, ArmConstants.L4pos),
        new RunIntakeOpenLoopCoral(m_IntakePivot, ArmConstants.CPivotScore),
        new Intakespit(m_intake).withTimeout(.5),
        new RunIntakeOpenLoopCoral(m_IntakePivot,ArmConstants.CPivotScore),
        new RunArmClosedLoop(m_Elevator, ArmConstants.L1pos)
      );
    SequentialCommandGroup ballGrab= new SequentialCommandGroup(
    new RunIntakeOpenLoop(m_IntakePivotA,.25),
    new IntakeNoteAutomatic(m_intakeA,1),
    new RunIntakeOpenLoop(m_IntakePivotA,.44)


    );
    SequentialCommandGroup CStow= new SequentialCommandGroup(
    new RunIntakeOpenLoopCoral(m_IntakePivot,ArmConstants.CPivotStow)
    );

    SequentialCommandGroup SpitBall= new SequentialCommandGroup(
    new DropA(m_intakeA).withTimeout(.5),
    new StopA(m_intakeA)
    

    );













    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        NamedCommands.registerCommand("L1", L1);
        NamedCommands.registerCommand("L2", L2);
        NamedCommands.registerCommand("L3", L3);
        NamedCommands.registerCommand("L4", L4);
        NamedCommands.registerCommand("INTRUN", CStow);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
       // joystick.x().whileTrue(new AlignCommand(drivetrain, m_Vision));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        joystick.x().whileTrue(new LimelightLateralAlignCommandX(drivetrain, 12.0));
        gamepad.b().onTrue(new RunIntakeOpenLoop(m_IntakePivotA, ArmConstants.APivotGrab));
        gamepad.y().onTrue(new RunIntakeOpenLoop(m_IntakePivotA, ArmConstants.APivotStow));
        gamepad.rightBumper().whileTrue(new Intakespit(m_intake));
        gamepad.leftBumper().whileTrue(new IntakeSUCK(m_intake));
        

        gamepad.povLeft().onTrue(   new RunIntakeOpenLoopCoral(m_IntakePivot,ArmConstants.CPivotGrab ));
        gamepad.povRight().onTrue(   new RunIntakeOpenLoopCoral(m_IntakePivot,ArmConstants.CPivotStow ));
        gamepad.povDown().onTrue(   new RunIntakeOpenLoopCoral(m_IntakePivot,ArmConstants.CPivotScore ));


        gamepad.povUp().onTrue(L1);    
        gamepad.a().onTrue( new RunArmClosedLoop(m_Elevator, ArmConstants.L1pos));    
        gamepad.x().onTrue( new RunArmClosedLoop(m_Elevator, ArmConstants.L3pos));      
        //gamepad.leftStick().onTrue(L4);     
        
        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */ 
        return autoChooser.getSelected();
    }
}

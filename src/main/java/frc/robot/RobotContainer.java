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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.CANConfig;
import frc.robot.commands.Climb;
import frc.robot.commands.Arm.RunArmClosedLoop;
import frc.robot.commands.Intake.IntakeSUCK;
import frc.robot.commands.Intake.Intakespit;
import frc.robot.commands.Intake.RunIntakeOpenLoopCoral;
import frc.robot.commands.Intake.IntakeSUCK;

import frc.robot.commands.IntakeA.RunIntakeOpenLoop;
import frc.robot.commands.limelight.LimeLeft;
import frc.robot.commands.limelight.LimeLight;
import frc.robot.generated.T16000MController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakePivotA;
import frc.robot.subsystems.climber;
import frc.robot.subsystems.intake;
//import frc.robot.subsystems.VisionSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;



public class RobotContainer {
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


    private final Elevator m_Elevator = new Elevator( CANConfig.ELEVATOR_FRONT, CANConfig.ELEVATOR_BACK);
    private final climber m_climber = new climber();

    private final IntakePivotA m_IntakePivotA = new IntakePivotA();
    final IntakePivot m_IntakePivot = new IntakePivot();
    private final intake m_intake = new intake(0);








    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final T16000MController joystick = new T16000MController(0);
    private final T16000MController joystick2 = new T16000MController(1);
    private final CommandXboxController gamepad = new CommandXboxController(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    


    ParallelCommandGroup L1= new ParallelCommandGroup(
        new RunArmClosedLoop(m_Elevator, ArmConstants.L1pos),
        new RunIntakeOpenLoopCoral(m_IntakePivot, ArmConstants.CPivotScore)
      );
      ParallelCommandGroup L2= new ParallelCommandGroup(
        new RunArmClosedLoop(m_Elevator, ArmConstants.L2pos),
        new RunIntakeOpenLoopCoral(m_IntakePivot, ArmConstants.CPivotScore)
      );
    ParallelCommandGroup L3= new ParallelCommandGroup(
        new RunArmClosedLoop(m_Elevator, ArmConstants.L3pos),
        new RunIntakeOpenLoopCoral(m_IntakePivot, ArmConstants.CPivotScore)

      );
      ParallelCommandGroup L4= new ParallelCommandGroup(
        new RunArmClosedLoop(m_Elevator, ArmConstants.L4pos),
        new RunIntakeOpenLoopCoral(m_IntakePivot, ArmConstants.CPivotScore)

      );



    
    SequentialCommandGroup CStow= new SequentialCommandGroup(
    new RunIntakeOpenLoopCoral(m_IntakePivot,ArmConstants.CPivotStow)
    );



    SequentialCommandGroup goLeft= new SequentialCommandGroup(
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric()),
        drivetrain.applyRequest(() ->
        drive.withVelocityX(0) 
            .withVelocityY(-1 * MaxSpeed) 
            .withRotationalRate(0) 
).withTimeout(.130),
drivetrain.applyRequest(() ->
drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    .withRotationalRate(joystick2.getLeftX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
)
    );
    SequentialCommandGroup goRight= new SequentialCommandGroup(
        drivetrain.applyRequest(() ->
        drive.withVelocityX(0) 
            .withVelocityY(1 * MaxSpeed) 
            .withRotationalRate(0) 
).withTimeout(.130),
drivetrain.applyRequest(() ->
drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    .withRotationalRate(joystick2.getLeftX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
)
    );









    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        NamedCommands.registerCommand("L1", L1);
        NamedCommands.registerCommand("L2", L2);
        NamedCommands.registerCommand("L3", L3);
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
                    .withRotationalRate(joystick2.getLeftX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );




       // joystick.x().whileTrue(new AlignCommand(drivetrain, m_Vision));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single frog.

        joystick2.L().whileTrue(new LimeLight(drivetrain, 4,0,"limelight-left"));
        joystick2.R().whileTrue(new LimeLight(drivetrain, 4,0,"limelight-right"));

//15.5
        //gamepad.b().onTrue(new RunIntakeOpenLoop(m_IntakePivotA, ArmConstants.APivotGrab));
        //gamepad.y().onTrue(new RunIntakeOpenLoop(m_IntakePivotA, ArmConstants.APivotStow));
        gamepad.rightBumper().whileTrue(new Intakespit(m_intake));
        gamepad.leftBumper().whileTrue(new IntakeSUCK(m_intake));

        



        gamepad.povRight().onTrue(   new RunIntakeOpenLoopCoral(m_IntakePivot,ArmConstants.CPivotStow ));
        gamepad.povDown().onTrue(   new RunIntakeOpenLoopCoral(m_IntakePivot,ArmConstants.CPivotScore ));

        gamepad.povUp().onTrue(L1);
        gamepad.a().onTrue(L2);    
        gamepad.x().onTrue(L3);  
        gamepad.leftStick().onTrue(L4);  

        gamepad.y().whileTrue(new Climb(m_climber, .5));
        gamepad.b().whileTrue(new Climb(m_climber, -.5));

        gamepad.rightTrigger().whileTrue(new RunIntakeOpenLoop(m_IntakePivotA, 9));
        gamepad.leftTrigger().whileTrue(new RunIntakeOpenLoop(m_IntakePivotA,0));

        
        // reset the field-centric heading on left bumper press    
         joystick.T().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */ 
        return autoChooser.getSelected();
    }
}

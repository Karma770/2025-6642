package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConfig;

// For Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.networktables.GenericEntry;

public class intakeA extends SubsystemBase {
    private final SparkMax left;
    private final SparkMax right;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController intakePID;

    private static final double STALL_CURRENT_LIMIT = 10.0; // Current threshold for stall condition

    // Shuffleboard for monitoring current
    private final ShuffleboardTab tab = Shuffleboard.getTab("Intake");
    private final GenericEntry currentEntry = tab.add("Left Motor Current", 0)
                                                .withWidget(BuiltInWidgets.kGraph)
                                                .getEntry();

    public intakeA() {
        left = new SparkMax(CANConfig.ALGAE_INTAKE_LEFT, MotorType.kBrushless);
        right = new SparkMax(CANConfig.ALGAE_INTAKE_RIGHT, MotorType.kBrushless);

        intakePID = left.getClosedLoopController();
        encoder = left.getEncoder();

        SparkMaxConfig Leftconfig = new SparkMaxConfig();
        Leftconfig.idleMode(IdleMode.kBrake);
        Leftconfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        Leftconfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(1.0)
            .d(0.5);

        SparkMaxConfig Rightconfig = new SparkMaxConfig();
        Rightconfig.inverted(true)
                   .idleMode(IdleMode.kBrake)
                   .follow(left, true);

        left.configure(Leftconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        right.configure(Rightconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void hold(double pos) {
        intakePID.setReference(pos, ControlType.kPosition);
    }

    public void stopIntake() {
        left.set(0);

        System.out.println("Intake stopped!");
    }

    public void runAtVelocity(double setpoint) {
        left.set(setpoint);
    }

    public void runOpenLoop(double supplier) {
        left.set(supplier);
    }

    public void autoIntake() {
        // Keep running unless the current exceeds the stall limit
        double current = getLeftMotorCurrent();

        if (current > STALL_CURRENT_LIMIT) {
            System.out.println("WARNING: Intake Stalled! Current: " + current + " A");
            stopIntake(); // Stop intake if current is too high (indicating a stall)
        } else {
            left.set(0.5); // Run the intake at 50% power (adjust as needed)
        }
        
        // Update the Shuffleboard and SmartDashboard with current values
        currentEntry.setDouble(current);
    }

    public boolean isCoral() {
        return true; // Placeholder for coral detection logic
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getOutputCurrent() {
        return left.getOutputCurrent();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public double getLeftMotorCurrent() {
        double current = left.getOutputCurrent();
        System.out.println("Left Spark MAX Current: " + current + " A");
        return current;
    }

    public double getTemp() {
        return left.getMotorTemperature();
    }

    @Override
    public void periodic() {
        // Update SmartDashboard every cycle with the current
        double current = getLeftMotorCurrent();
        SmartDashboard.putNumber("Left Motor Current", current);
        currentEntry.setDouble(current);
         if (current > STALL_CURRENT_LIMIT) {
            System.out.println("WARNING: Intake Stalled! Current: " + current + " A");
            stopIntake(); }
    }
}

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConfig;

public class intake extends SubsystemBase{
    /*We need methods to intake and stop when Coral is detected, feed to shooter, reverse intake and feed manually.
     * Common wisdom says that the intake should run at 2x drive speed.
     */
    private final SparkFlex left;
    private final SparkFlex right;

    private final RelativeEncoder encoder;
    private final DigitalInput optic;

    private final SparkClosedLoopController intakePID;
    private final ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
    private final GenericEntry opticEntry = tab.add("Optic", 0)
                                                .withWidget(BuiltInWidgets.kBooleanBox)
                                                .getEntry();
    private final GenericEntry PosEntry = tab.add("POS", 0)
                                                .getEntry();

    public intake(int Optic) {

        left = new SparkFlex (CANConfig.CORAL_RUN_LEFT ,MotorType.kBrushless);
        right = new SparkFlex (CANConfig.CORAL_RUN_RIGHT, MotorType.kBrushless);
        optic = new DigitalInput(0);
  

        intakePID = left.getClosedLoopController();
        encoder = left.getEncoder();
        SparkMaxConfig Leftconfig = new SparkMaxConfig();
        Leftconfig
            .idleMode(IdleMode.kBrake);
        Leftconfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        Leftconfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(1.0)
            .d(.5);
        
                    SparkMaxConfig Rightconfig = new SparkMaxConfig();
        Rightconfig
            .inverted(true )
            .idleMode(IdleMode.kBrake)
            .follow(left, true);

        left.configure(Leftconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        right.configure(Rightconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            


    }

    public void hold(double pos) {
        intakePID.setReference(pos, ControlType.kPosition);
    }
    public void stopIntake (double speed)
    {
        left.set(0);
    }
    public void runAtVelocity(double setpoint) {

        left.set(setpoint);
    }

    public void runOpenLoop(double supplier) {

        left.set(supplier);
        
    }

    public void autoIntake() {
        if(!isCoral()){
            intakePID.setReference(0.05, ControlType.kVelocity);
        }
        else {
            hold(encoder.getPosition());
        }
    }

    public boolean isCoral() {
        return false;
       // return optic.get();
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

    public double getTemp() {
        return left.getMotorTemperature();
    }    
    public void periodic() {
        boolean opticTF = isCoral();
        opticEntry.setBoolean(opticTF);
        double POS = encoder.getPosition();
        PosEntry.setDouble(POS);
    }
    
    
}
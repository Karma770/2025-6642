package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;








public class Elevator extends SubsystemBase{

    /*We need methods to run the arm to postions using a rev throughbore encoder in absolute mode.
     * It is needed to read arm position and to move the arm.
     */

    public final SparkMax LeftMotor;
    public final SparkMax RightMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController armPID;

    private final ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
    private final GenericEntry currentEntry = tab.add("position", 0)
                                                .withWidget(BuiltInWidgets.kGraph)
                                                .getEntry();



    public Elevator (int elevatorFront, int elevatorBack) {

        LeftMotor = new SparkMax(CANConfig.ELEVATOR_BACK, MotorType.kBrushless);
        RightMotor = new SparkMax(CANConfig.ELEVATOR_FRONT, MotorType.kBrushless);

        SparkMaxConfig Leftconfig = new SparkMaxConfig();
Leftconfig
    .inverted(true)
    .idleMode(IdleMode.kBrake);
Leftconfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);
Leftconfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(.50)
    .d(.75);

            SparkMaxConfig Rightconfig = new SparkMaxConfig();
Rightconfig
    .inverted(true )
    .idleMode(IdleMode.kBrake)
    .follow(LeftMotor, true);

    
LeftMotor.configure(Leftconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
RightMotor.configure(Rightconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = LeftMotor.getEncoder();
        armPID = LeftMotor.getClosedLoopController();




    }

    public void runOpenLoop(double supplier) {
        if(getPos() >= ArmConstants.kUpperLimit) {
            LeftMotor.set(0);
            System.out.println("¡TOO HIGH! ¡UPPER LIMIT!");
        }
        else if(getPos() <= ArmConstants.kLowerLimit) {
            LeftMotor.set(0);
            System.out.println("¡TOO LOW! ¡LOWER LIMIT!");
        }
        else {
            LeftMotor.set(supplier);
            
        }
    }

    public void hold() {
        //armPID.setReference(encoder.getPosition(), ControlType.kPosition);
         if(getPos() < ArmConstants.kUpperLimit) {
        LeftMotor.set(0.0);
         }
    }
    public void stopArm(double speed){

        LeftMotor.set(0);
       


}
    public void runToPosition(double setpoint) {
        System.out.println("Arm Current Position");
        System.out.println(getPos());
        System.out.println(setpoint);
        //these are included safety measures. not necessary, but useful
        if(getPos() >= ArmConstants.kUpperLimit) {
            LeftMotor.set(-1);
            System.out.println("¡TOO HIGH! ¡UPPER LIMIT!");
            System.out.println(getPos());
        }
        else if(getPos() <= ArmConstants.kLowerLimit) {
            LeftMotor.set(1);
            System.out.println("¡TOO LOW! ¡LOWER LIMIT!");
            System.out.println(getPos());
        }
        else{
            armPID.setReference(setpoint, ControlType.kPosition);
            

        }
    }

    public double getPos() {
        return encoder.getPosition();
    }

 public void periodic() {
        // Update SmartDashboard every cycle with the current
        double current = getPos();
        SmartDashboard.putNumber("Position", current);
        currentEntry.setDouble(current);

    }
    
}
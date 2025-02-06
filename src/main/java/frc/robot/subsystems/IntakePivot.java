package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.ExternalEncoderConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;








public class IntakePivot extends SubsystemBase{

    /*We need methods to run the arm to postions using a rev throughbore encoder in absolute mode.
     * It is needed to read arm position and to move the arm.
     */
    public final SparkMax pivotMotor;
    private final AbsoluteEncoder encoder;
    private final SparkClosedLoopController armPID;


    public IntakePivot () {

        pivotMotor = new SparkMax(CANConfig.CORAL_PIVOT, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
config
    .idleMode(IdleMode.kBrake);
config.encoder
    .positionConversionFactor(1000)
    .velocityConversionFactor(1000);
config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(1.0, 0.0, 0.0);
    
pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = pivotMotor.getAbsoluteEncoder();
        armPID = pivotMotor.getClosedLoopController();




    }

    public void runOpenLoop(double supplier) {
        if(getPos() >= ArmConstants.kUpperLimit) {
            pivotMotor.set(0);
            System.out.println("¡TOO HIGH! ¡UPPER LIMIT!");
        }
        else if(getPos() <= ArmConstants.kLowerLimit) {
            pivotMotor.set(0);
            System.out.println("¡TOO LOW! ¡LOWER LIMIT!");
        }
        else {
            pivotMotor.set(supplier);
            
        }
    }

    public void hold() {
        //armPID.setReference(encoder.getPosition(), ControlType.kPosition);
         if(getPos() < ArmConstants.kUpperLimit) {
        pivotMotor.set(0.03);
         }
    }
    public void stopArm(double speed){

        pivotMotor.set(0);
       


}
    public void runToPosition(double setpoint) {
        System.out.println("Arm Current Position");
        System.out.println(getPos());
        System.out.println(setpoint);
        //these are included safety measures. not necessary, but useful
        if(getPos() >= ArmConstants.kUpperLimit) {
            pivotMotor.set(0);
            System.out.println("¡TOO HIGH! ¡UPPER LIMIT!");
            System.out.println(getPos());
        }
        else if(getPos() <= ArmConstants.kLowerLimit) {
            pivotMotor.set(0);;
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
    
}
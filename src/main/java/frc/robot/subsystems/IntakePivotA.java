package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;








public class IntakePivotA extends SubsystemBase{

    /*We need methods to run the arm to postions using a rev throughbore encoder in absolute mode.
     * It is needed to read arm position and to move the arm.
     */

    public final SparkMax LeftMotor;
    private final AbsoluteEncoder encoder;
    private final SparkClosedLoopController armPID;


    public IntakePivotA () {

        LeftMotor = new SparkMax(CANConfig.ALGAE_PIVOT, MotorType.kBrushless);
        encoder = LeftMotor.getAbsoluteEncoder();
        armPID = LeftMotor.getClosedLoopController();


        SparkMaxConfig Leftconfig = new SparkMaxConfig();
Leftconfig
    .inverted(false)
    .smartCurrentLimit(40)
    .idleMode(IdleMode.kBrake);
Leftconfig.absoluteEncoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);
Leftconfig.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .p(1.25)
    .d(0);

            SparkMaxConfig Rightconfig = new SparkMaxConfig();
Rightconfig
    .idleMode(IdleMode.kBrake)
    .follow(LeftMotor, true);

    
LeftMotor.configure(Leftconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);




    }

    public void runOpenLoop(double supplier) {
        if(getPos() >= 1) {
            LeftMotor.set(0);
            System.out.println("¡TOO HIGH! ¡UPPER LIMIT!");
        }
        else if(getPos() <= 0) {
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
        LeftMotor.set(0.03);
         }
    }
    public void stopArm(double speed){

        LeftMotor.set(0);
       


}
    public void runToPosition(double setpoint) {
        System.out.println("Arm Current Position");
        System.out.println(getPos());
        System.out.println(setpoint);
   
            armPID.setReference(setpoint, ControlType.kPosition);
        


        
    }

    public double getPos() {
        return encoder.getPosition();
    }
    
}
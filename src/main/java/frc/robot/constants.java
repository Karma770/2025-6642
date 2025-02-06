package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

public class constants {
    
    public static class CANConfig {
        public static final int ELEVATOR_FRONT = 30;
        public static final int ELEVATOR_BACK = 31;
        public static final int ALGAE_PIVOT = 1;
        public static final int CORAL_PIVOT = 2;
        public static final int ALGAE_INTAKE = 3;
        public static final int CORAL_INTAKE = 4;

      }

      public static class SystemConfig {
        public static final double ELEVATOR_SPEED = 0.5;
        public static final double PIVOT_SPEED = 0.5;
        public static final double ALGAE_INTAKE_SPEED = 0.5;
        public static final double CORAL_INTAKE_SPEED = 0.5;
      }

        public static class VisionConstants {
    public static final String LIMELIGHT_NAME = "";
    public static final Distance LIMELIGHT_LENS_HEIGHT = Distance.ofBaseUnits(8, Inches);
    public static final Angle LIMELIGHT_ANGLE = Angle.ofBaseUnits(0, Degrees);

    public static final Distance REEF_APRILTAG_HEIGHT = Distance.ofBaseUnits(6.875, Inches);
    public static final Distance PROCCESSOR_APRILTAG_HEIGHT = Distance.ofBaseUnits(45.875, Inches);
    public static final Distance CORAL_APRILTAG_HEIGHT = Distance.ofBaseUnits(53.25, Inches);
  }
      public static class ArmConstants  {

        public static final double kUpperLimit = 1;
        public static final double kLowerLimit = 0.5;
                public static final double kTolearance = 0.01;
        public static final double kManualSpeed = 0.3;

      }


    }

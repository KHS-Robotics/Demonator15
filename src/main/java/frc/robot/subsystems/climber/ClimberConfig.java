package frc.robot.subsystems.climber;

public class ClimberConfig {

 protected final class ClimberMotorConfig {
     public static final double kClimberP = 0.0;
     public static final double kClimberI = 0.0;
     public static final double kClimberD = 0.0;

     public static final double kClimberKG = 0.0;
     public static final double kClimberGearing = 0.0;


//(credit to griffin for giving emotional support)

}

 protected final class ElevatorSetpoints{
     public static final double STOW = 0;
     public static final double L1 = 0;
     public static final double L2 = 0;
     public static final double L3 = 0;
    }
 
 protected final class ClimberHookAngle {
     public static final int DeployedHookAngle = 0;
     public static final int RetractedHookAngle = 0; 
     public static final double stowedHookAngle = 0;  
 } 

 public static final double kClimberAbsoluteEncoderMinVoltage = 0.0;
 public static final double kClimberAbsoluteEncoderMaxVoltage = 0.0;
}
// package frc.robot.subsystems;

// import com.revrobotics.PersistMode;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class Climber extends SubsystemBase {
//     private static SparkMax climberMotor = new SparkMax(Constants.kClimberMotorID, MotorType.kBrushless);
//     private static SparkMaxConfig climberConfig = new SparkMaxConfig();

//     public Climber() {
//         climberConfig.idleMode(IdleMode.kBrake);
//         climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
//     }
// }
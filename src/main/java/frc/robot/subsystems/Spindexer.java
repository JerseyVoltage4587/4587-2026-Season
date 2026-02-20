package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spindexer extends SubsystemBase {
    
    // private static SparkMax spindexerMotor = new SparkMax(Constants.kSpindexerMotorID, MotorType.kBrushless);
    // private static SparkMaxConfig spindexerConfig = new SparkMaxConfig();

    // private TalonFX spindexerMotor = new TalonFX(Constants.kSpindexerMotorID);
    
    private static SparkMax fromIntakeMotor = new SparkMax(Constants.kFromIntakeMotorID, MotorType.kBrushless);
    private static SparkMaxConfig fromIntakeConfig = new SparkMaxConfig();
    
    private static SparkMax toShooterMotor = new SparkMax(Constants.kToShooterMotorID, MotorType.kBrushless);
    private static SparkMaxConfig toShooterConfig = new SparkMaxConfig();

    public Spindexer() {
        
        // spindexerMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withNeutralMode(NeutralModeValue.Coast));
        // spindexerMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(30));
        
        fromIntakeConfig.idleMode(IdleMode.kCoast);
        fromIntakeConfig.smartCurrentLimit(5, 35);
        //ADD CURRENY LIMIT
        toShooterConfig.idleMode(IdleMode.kCoast);
        toShooterConfig.smartCurrentLimit(5, 35);

        // fromIntakeMotor.configure(fromIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        toShooterMotor.configure(toShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    
    private void spindexerSpeed() {
        fromIntakeMotor.set(0.1);
    }

    private void zeroSpindexerMotor() {
        fromIntakeMotor.set(0);
    }

    private void toShooterSpeed() {
        toShooterMotor.set(0.1);
    }

    private void zeroToShooterMotor() {
        toShooterMotor.set(0);
    }

    public Command spindexerSpeedCommand() {
        return runEnd(() -> spindexerSpeed(), () -> zeroSpindexerMotor());
    }

    public Command toShooterSpeeCommand() {
        return runEnd(() -> toShooterSpeed(), () -> zeroToShooterMotor());
    }

}

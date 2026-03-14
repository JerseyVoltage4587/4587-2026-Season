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

public class Indexer extends SubsystemBase {
        
    private static SparkMax mainBeltMotor = new SparkMax(Constants.kMainBeltMotorID, MotorType.kBrushless);
    private static SparkMaxConfig mainBeltConfig = new SparkMaxConfig();
    
    private static SparkMax feedMotor = new SparkMax(Constants.kFeedMotorID, MotorType.kBrushless);
    private static SparkMaxConfig feedConfig = new SparkMaxConfig();

    public Indexer() {
                
        mainBeltConfig.idleMode(IdleMode.kCoast);
        mainBeltConfig.smartCurrentLimit(5, 35);
        
        feedConfig.idleMode(IdleMode.kCoast);
        feedConfig.smartCurrentLimit(5, 35);

        mainBeltMotor.configure(mainBeltConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        feedMotor.configure(feedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    
    private void beltSpeed() {
        mainBeltMotor.set(0.1);
    }

    private void zeroMainBeltMotor() {
        mainBeltMotor.set(0);
    }

    private void feedSpeed() {
        feedMotor.set(0.1);
    }

    private void zeroFeedMotor() {
        feedMotor.set(0);
    }

    public Command beltSpeedCommand() {
        return runEnd(() -> beltSpeed(), () -> zeroMainBeltMotor());
    }

    public Command feedSpeedCommand() {
        return runEnd(() -> feedSpeed(), () -> zeroFeedMotor());
    }

}

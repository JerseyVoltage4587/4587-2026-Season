package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
    
    // private static TalonFX feedMotor = new TalonFX(Constants.kFeedMotorID);
    // private static TalonFXConfiguration feedConfig = new TalonFXConfiguration();

    public Indexer() {
                
        mainBeltConfig.idleMode(IdleMode.kCoast);
        mainBeltConfig.inverted(true);
        // mainBeltConfig.smartCurrentLimit(5, 35);
        
        // feedConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        // feedConfig.smartCurrentLimit(5, 35);

        mainBeltMotor.configure(mainBeltConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        // feedMotor.getConfigurator().apply(feedConfig);
    }
    
    @Override
    public void periodic()
    {

    }
    private void beltSpeed() {
        mainBeltMotor.set(0.5);
    }

    private void zeroMainBeltMotor() {
        mainBeltMotor.set(0);
    }

    // private void feedSpeed() {
    //     feedMotor.set(0.5);
    // }

    // private void zeroFeedMotor() {
    //     feedMotor.set(0);
    // }

    // private void fullIndexer() {
    //     feedMotor.set(0.5);
    //     mainBeltMotor.set(0.5);
    // }

    // private void zeroIndexerMotors() {
    //     feedMotor.set(0);
    //     mainBeltMotor.set(0);
    // }

    public Command beltSpeedCommand() {
        return runEnd(() -> beltSpeed(), () -> zeroMainBeltMotor());
    }

    // public Command feedSpeedCommand() {
    //     return runEnd(() -> feedSpeed(), () -> zeroFeedMotor());
    // }

    // public Command fullIndexerCommand() {
    //     return runEnd(() -> fullIndexer(), () -> zeroIndexerMotors());
    // }

}

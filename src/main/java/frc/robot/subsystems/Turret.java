package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase
{
    private static SparkMax turretMotor = new SparkMax(Constants.kTurretMotorID, MotorType.kBrushless);
    private static SparkMaxConfig turretConfig = new SparkMaxConfig();

    public Turret() {
        turretConfig.idleMode(IdleMode.kBrake);
        turretConfig.smartCurrentLimit(10, 30); //TEST AAAAH
        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
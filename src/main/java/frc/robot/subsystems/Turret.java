package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase
{
    private static SparkMax turretMotor = new SparkMax(Constants.kTurretMotorID, MotorType.kBrushless);
    private static SparkMaxConfig turretConfig = new SparkMaxConfig();
    private static PIDController turretPIDController = new PIDController(
        TurretConstants.kTurretP, TurretConstants.kTurretI, TurretConstants.kTurretD
    );
    
    public Turret() {        
        turretConfig.idleMode(IdleMode.kBrake);
        // turretConfig.smartCurrentLimit(10, 30); //TEST AAAAH
        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        turretMotor.getEncoder().setPosition(0);
    }

    private void goToPosition(double pos) {
        turretMotor.set(turretPIDController.calculate(turretMotor.getEncoder().getPosition(), pos));
    }

    private void goToDegrees(double deg) {
        goToPosition(deg * (TurretConstants.kMaxTurretEncoderValue / 360));
    }

    private void preventTangleAtMaxAngle() {
        if (turretMotor.getEncoder().getPosition() > TurretConstants.kMaxTurretEncoderValue) {
            goToPosition(0);
        }
    }

    private void preventTangleAtMinAngle() {
        if (turretMotor.getEncoder().getPosition() < 0) {
            goToPosition(TurretConstants.kMaxTurretEncoderValue);
        }
    }

    private void turretClockwise() {
        turretMotor.set(0.25);
    }

    private void turretCClockwise() {
        turretMotor.set(-0.25);
    }

    private void zeroTurret() {
        turretMotor.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // preventTangleAtMaxAngle();
        // preventTangleAtMinAngle();
    }

    public Command turretGoToDegrees(DoubleSupplier deg) {
        return runEnd(() -> goToDegrees(deg.getAsDouble()), () -> zeroTurret());
    }

    public Command turretClockwiseCommand()  {
        return runEnd(() -> turretClockwise(), () -> zeroTurret());
    }
    
    public Command turretCClockwiseCommand()  {
        return runEnd(() -> turretCClockwise(), () -> zeroTurret());
    }

    
}
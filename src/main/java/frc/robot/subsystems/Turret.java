package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
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
        turretPIDController.setTolerance(.1);
    }

    private void goToPosition(double pos) {
        // double turretSpeed = turretPIDController.calculate(turretMotor.getEncoder().getPosition(), pos);
        // SmartDashboard.putNumber("turretspeed", turretSpeed);
        // if (Math.abs(turretSpeed)  < (RobotConstants.kDeadBand)) {
        //     turretMotor.set(0);
        // } else {
        turretMotor.set(MathUtil.applyDeadband(turretPIDController.calculate(turretMotor.getEncoder().getPosition(), pos), RobotConstants.kDeadBand * 4));
    }

    private void goToDegrees(double deg) {
        goToPosition((deg / 180) * TurretConstants.kMinTurretEncoderValue);
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
        turretMotor.set(0.5);
    }

    private void turretCounterClockwise() {
        turretMotor.set(-0.5);
    }

    private void zeroTurret() {
        turretMotor.set(0);
    }

    private void resetTurretGyro() {
        turretMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        SmartDashboard.putNumber("Turret Encoder", turretMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Turret Motor Speed", turretMotor.get());  
        SmartDashboard.putNumber("Error Tolerance", turretPIDController.getErrorTolerance());
        // preventTangleAtMaxAngle();
        // preventTangleAtMinAngle();
    }

    public Command turretResetGyroCommand() {
        return runOnce(() -> resetTurretGyro());
    }

    public Command turretGoToDegreesCommand(DoubleSupplier deg) {
        return runEnd(() -> goToDegrees(deg.getAsDouble()), () -> zeroTurret());
    }

    public Command turretGoToPosCommand(DoubleSupplier pos) {
        return runEnd(() -> goToPosition(pos.getAsDouble()), () -> zeroTurret());
    }

    public Command turretClockwiseCommand()  {
        return runEnd(() -> turretClockwise(), () -> zeroTurret());
    }
    
    public Command turretCounterClockwiseCommand()  {
        return runEnd(() -> turretCounterClockwise(), () -> zeroTurret());
    }

    
}
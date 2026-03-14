// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final SwerveSubsystem m_swervesubsystem = new SwerveSubsystem();
  private static final Shooter m_shooter = new Shooter();

  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick j = new Joystick(0);
  private final Joystick k = new Joystick(1);
  
  private JoystickButton jButtonY, jButtonB, jButtonA, jButtonX, jLeftBumper, jRightBumper, jLeftTrigger, jRightTrigger, 
    jMinusButton, jPlusButton, jHouseButton, jCircleButton;
  
  private JoystickButton kButtonY, kButtonB, kButtonA, kButtonX, kLeftBumper, kRightBumper, kLeftTrigger, kRightTrigger, 
    kMinusButton, kPlusButton, kLeftStickButton, kRightStickButton, kHouseButton, kCircleButton;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    jButtonY = new JoystickButton(j, 1); 
    jButtonB = new JoystickButton(j, 2); // Future Assignment: Lower Climber
    jButtonA = new JoystickButton(j, 3); // Final Assignment: Robot Relative Joystick Controls
    jButtonX = new JoystickButton(j, 4); // Future Assignment: Raise Climber
    jLeftBumper = new JoystickButton(j, 5); // Future Assignment: Retract Intake
    jRightBumper = new JoystickButton(j, 6); // Future Assignment: Release Intake
    jLeftTrigger = new JoystickButton(j, 7); // Future Assignment: Pass Balls to Trench Tag
    jRightTrigger = new JoystickButton(j, 8); // Future Assignment: Auto Aim & Shoot Fuel
    jMinusButton = new JoystickButton(j, 9);
    jPlusButton = new JoystickButton(j, 10);
    jHouseButton = new JoystickButton(j, 13);
    jCircleButton = new JoystickButton(j, 14);


    kButtonY = new JoystickButton(k, 1);
    kButtonB = new JoystickButton(k, 2); // Current Assignment: Lower Hood
    kButtonA = new JoystickButton(k, 3); // Current Assignment: Set Shooter Speed
    kButtonX = new JoystickButton(k, 4); // Current Assignment: Raise Hood
    kLeftBumper = new JoystickButton(k, 5); // Current Assignment: Lower Shooter Speed
    kRightBumper = new JoystickButton(k, 6); // Current Assignment: Raise Shooter Speed
    kLeftTrigger = new JoystickButton(k, 7);
    kRightTrigger = new JoystickButton(k, 8);
    kMinusButton = new JoystickButton(k, 9);
    kPlusButton = new JoystickButton(k, 10);
    kLeftStickButton = new JoystickButton(k, 11);
    kRightStickButton = new JoystickButton(k, 12);
    kHouseButton = new JoystickButton(k, 13);
    kCircleButton = new JoystickButton(k, 14);

    m_swervesubsystem.setDefaultCommand(m_swervesubsystem.DriveCommand(
      () -> j.getRawAxis(1),
      () -> j.getRawAxis(0),
      () -> j.getRawAxis(2),
      () -> jButtonA.getAsBoolean()
    ));

    kRightBumper.onTrue(m_shooter.fasterShootCommand());
    kLeftBumper.onTrue(m_shooter.slowerShootCommand());
    kButtonA.whileTrue(m_shooter.standardShooterSpeedCommand());
    kButtonB.whileTrue(m_shooter.hoodBackwardCommand());
    kButtonX.whileTrue(m_shooter.hoodForwardCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  // public Command autoPathCommand(String name) {
  //   try {
  //     // List<PathPlannerPath> auto = PathPlannerAuto.getPathGroupFromAutoFile(name);

  //     return AutoBuilder.buildAuto(name);
  //   } catch (Exception e) {
  //     DriverStation.reportError(e.getMessage(), e.getStackTrace());
  //     return Commands.none();
  //   }
  // }
}

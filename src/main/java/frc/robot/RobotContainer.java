// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import javax.print.attribute.standard.MediaTray;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Drivetrain.SwerveSubsystem;
import pabeles.concurrency.IntOperatorTask.Min;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final SwerveSubsystem m_swerve = new SwerveSubsystem();
  private static final Shooter m_shooter = new Shooter();
  private static final Turret m_turret = new Turret();
  private static final Hood m_hood = new Hood();
  private static final Vision m_vision = new Vision(m_swerve::addVisionMeasurement);
  private static final Indexer m_indexer = new Indexer();
  private static final Intake m_intake = new Intake();
  private static final Feed m_feed = new Feed();
  public String visionTarget = "";

  private SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick j = new Joystick(0);
  private final Joystick k = new Joystick(1);
  
  private JoystickButton jButtonY, jButtonB, jButtonA, jButtonX, jLeftBumper, jRightBumper, jLeftTrigger, jRightTrigger, 
    jMinusButton, jPlusButton, jHouseButton, jCircleButton;
  
  private JoystickButton kButtonY, kButtonB, kButtonA, kButtonX, kLeftBumper, kRightBumper, kLeftTrigger, kRightTrigger, 
    kMinusButton, kPlusButton, kLeftStickButton, kRightStickButton, kHouseButton, kCircleButton;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    
    // Configure the trigger bindings
    NamedCommands.registerCommand("IndexingAndShooting", indexerAndShootingCommandGroup());
    configureBindings();
    CameraServer.startAutomaticCapture();
    

    new Thread(() -> {
      try {
        m_swerve.InitGyro();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
      } catch (Exception e) {
      }
   }).start();
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

    // Comments Key:
    // Current Assignment: The current command that is assigned to that button, hoping to change soon
    // Future Assignment: The final command we want to assign to that button
    // Final Assignment: The current and future command assigned to that button

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
    kButtonB = new JoystickButton(k, 2); // Commented Out Current Assignment: Lower Hood
    kButtonA = new JoystickButton(k, 3); // Current Assignment: Set Shooter Speed
    kButtonX = new JoystickButton(k, 4); // Commented Out Current Assignment: Raise Hood
    kLeftBumper = new JoystickButton(k, 5); // Current Assignment: Lower Shooter Speed
    kRightBumper = new JoystickButton(k, 6); // Current Assignment: Raise Shooter Speed
    kLeftTrigger = new JoystickButton(k, 7); // Current Assignment: Run Intake
    kRightTrigger = new JoystickButton(k, 8); // Current Assignment: Run Indexer
    kMinusButton = new JoystickButton(k, 9); // Current Assignment: Bring Release Intake In
    kPlusButton = new JoystickButton(k, 10); // Current Assignment: Bring Release Intake Out
    kLeftStickButton = new JoystickButton(k, 11);
    kRightStickButton = new JoystickButton(k, 12);
    kHouseButton = new JoystickButton(k, 13);
    kCircleButton = new JoystickButton(k, 14);

    visionTarget = null;
    chooseVisionTarget(() -> DriverStation.getAlliance().toString().contains(Alliance.Red.toString()));

    m_swerve.setDefaultCommand(m_swerve.DriveCommand(
      () -> j.getRawAxis(1),
      () -> j.getRawAxis(0),
      () -> j.getRawAxis(2)
    ));


    jButtonX.onTrue(m_swerve.ZeroGyroCommand());
    // m_turret.setDefaultCommand(m_turret.turretGoToDegrees(Math.toDegrees(turretTargetAngle())));
    // kRightTrigger.whileTrue(m_turret.turretGoToDegrees(() -> Math.toDegrees(turretTargetAngle())));

    // m_shooter.setDefaultCommand(m_shooter.hoodGoToDegreesCommand(hoodTarget()));
    // kLeftTrigger.whileTrue(m_hood.hoodGoToDegreesCommand(() -> hoodTarget()));

    // kRightBumper.onTrue(m_shooter.fasterShootCommand());
    // kLeftBumper.onTrue(m_shooter.slowerShootCommand());
    // kButtonA.whileTrue(m_shooter.standardShooterSpeedCommand());
    // kButtonB.whileTrue(m_hood.hoodBackwardCommand());
    // kButtonX.whileTrue(m_hood.hoodForwardCommand());

    kButtonY.whileTrue(m_turret.turretGoToPosCommand(null));

    kRightBumper.whileTrue(m_turret.turretClockwiseCommand());
    kLeftBumper.whileTrue(m_turret.turretCounterClockwiseCommand());

    // kRightTrigger.whileTrue(m_feed.feedSpeedCommand());
    // kButtonY.whileTrue(m_indexer.beltSpeedCommand());
    // kCircleButton.whileTrue(m_shooter.bangBangCommand());
    kHouseButton.whileTrue(m_shooter.bangBangBaselineCommand(shooterTarget()));

    // kPlusButton.whileTrue(m_intake.bringReleaseIntakeOutCommand());
    // kMinusButton.whileTrue(m_intake.bringReleaseIntakeInCommand());
    // kLeftTrigger.whileTrue(m_intake.RunIntakeBallCommand());

    jRightBumper.whileTrue(m_intake.bringReleaseIntakeOutCommand());
    jLeftBumper.whileTrue(m_intake.bringReleaseIntakeInCommand());
    jRightTrigger.whileTrue(m_intake.RunIntakeBallCommand());
    // jLeftTrigger.whileTrue(outtakeBallsCommandGroup());
    jLeftTrigger.whileTrue(indexerAndShootingCommandGroup());
  }

  public ParallelCommandGroup indexerAndShootingCommandGroup() {
    return new ParallelCommandGroup(
      m_shooter.bangBangCommand(),
      new WaitCommand(1.5).andThen(m_feed.feedSpeedCommand()),
      new WaitCommand(1.5).andThen(m_indexer.beltSpeedCommand())
    );
  }

  public ParallelCommandGroup outtakeBallsCommandGroup() {
    return new ParallelCommandGroup(
      m_indexer.beltBackwardsCommand(),
      m_intake.RunOuttakeBallCOmmand()
    );
  }

  public ParallelCommandGroup autonPreloadCommandGroup() {
    return new ParallelCommandGroup(
      m_shooter.bangBangCommand(),
      new WaitCommand(1.5).andThen(m_feed.feedSpeedCommand()),
      new WaitCommand(2.5).andThen(m_indexer.beltSpeedCommand()),
      new WaitCommand(3.5).andThen(m_intake.RunOuttakeBallCOmmand())
    );
  }

  public void chooseVisionTarget(Supplier<Boolean> redAlliance) {
    // Chooses vision target based on distance from Alliance Wall (x) and distance from left field wall (y) 
    
    if (redAlliance.get()) {
      if (m_swerve.getPose().getX() >= 11.57) {
        visionTarget = "Red Hub";
        } else if (m_swerve.getPose().getX() >= 4.03 && m_swerve.getPose().getY() > 4.035) {
          visionTarget = "Red Depot Trench";
        } else if (m_swerve.getPose().getX() >= 4.03 && m_swerve.getPose().getY() < 4.035) {
          visionTarget = "Red Outpost Trench";
        } else if (m_swerve.getPose().getX() < 4.03 && m_swerve.getPose().getY() > 4.035) {
          visionTarget = "Blue Outpost Trench";
        } else if (m_swerve.getPose().getX() < 4.03 && m_swerve.getPose().getY() < 4.035) {
          visionTarget = "Blue Depot Trench";
        } else {
          visionTarget = null;
        }
      } else {
        if (m_swerve.getPose().getX() <= 4.03) {
          visionTarget = "Blue Hub";
        } else if (m_swerve.getPose().getX() <= 11.57 && m_swerve.getPose().getY() < 4.035) {
          visionTarget = "Blue Depot Trench";
        } else if (m_swerve.getPose().getX() <= 11.57 && m_swerve.getPose().getY() > 4.035) {
          visionTarget = "Blue Outpost Trench";
        } else if (m_swerve.getPose().getX() > 11.57 && m_swerve.getPose().getY() < 4.035) {
          visionTarget = "Red Outpost Trench";
        } else if (m_swerve.getPose().getX() > 11.57 && m_swerve.getPose().getY() > 4.035) {
          visionTarget = "Red Depot Trench";
        } else {
          visionTarget = null;
        }
      }

    }

    // Finds the angle to a Translation 2d target using the x and y coords of the robot pose
    public double translationAngleToTarget(Translation2d target) {
      double xDistanceToTarget = target.getX() - m_swerve.getPose().getX();
      double yDistanceToTarget = target.getY() - m_swerve.getPose().getY();
      
      return Math.toDegrees(Math.atan2(xDistanceToTarget, yDistanceToTarget));
    }
    
    public double turnBackToBump() {
      double turnGyroToBump = 0.0;
      if(visionTarget.equals("Blue Hub") || visionTarget.equals("Red Outpost Trench") || visionTarget.equals("Red Depot Trench"))
      {
        if(DriverStation.getAlliance().equals(DriverStation.Alliance.Red))
        {
          turnGyroToBump = 0;
        }
        else
        {
          turnGyroToBump = 180.0;
        }
      }
      else if(visionTarget.equals("Red Hub") || visionTarget.equals("Blue Outpost Trench") || visionTarget.equals("Blue Depot Trench"))
      {
        if(DriverStation.getAlliance().equals(DriverStation.Alliance.Red))
        {
          turnGyroToBump = 180.0;
        }
        else
        {
          turnGyroToBump = 0.0;
        }
      }
      return turnGyroToBump;
    }

    public double clipAngle(double x) {
      while (x > 180) {
        x -= 360;
      }
      while (x < -180) {
        x += 360;
      }

      return x;
    }

    public double turretTargetAngle() {

      double returnAngle;

      switch (visionTarget) {
        case "Red Hub": returnAngle = translationAngleToTarget(Constants.kRedHubCoord); break;
        case "Blue Hub": returnAngle = translationAngleToTarget(Constants.kBlueHubCoord); break;
        case "Blue Depot Trench": returnAngle = translationAngleToTarget(Constants.kBlueDepotTrenchCoord); break;
        case "Blue Outpost Trench": returnAngle = translationAngleToTarget(Constants.kBlueOutpostTrenchCoord); break;
        case "Red Depot Trench": returnAngle = translationAngleToTarget(Constants.kRedDepotTrenchCoord); break;
        case "Red Outpost Trench": returnAngle = translationAngleToTarget(Constants.kRedOutpostTrenchCoord); break;
        default: returnAngle = 0; break;
      } 
      
      return clipAngle(returnAngle);
  }

  public double shooterTarget() {
    double shooterSpeedTarget;

    if(visionTarget.equals("Red Hub") || visionTarget.equals("Blue Hub")) {
      shooterSpeedTarget = ShooterConstants.kMinMaxShooterSpeedDifference * ((Constants.kMaxDistanceInMeters - m_swerve.getPose().getX()) / Constants.kMaxDistanceInMeters) + ShooterConstants.kMinShooterMotorSpeed;
    } 
    else {
      shooterSpeedTarget = 0.8;
    }
    
    return shooterSpeedTarget;
  }

  public double hoodTarget(Translation2d target) {
    double hoodAngleTarget;
    double xDistanceToTarget = target.getX() - m_swerve.getPose().getX();
    double yDistanceToTarget = target.getY() - m_swerve.getPose().getY();

    double diagDistanceToTarget = Math.sqrt(Math.pow(xDistanceToTarget, 2) + Math.pow(yDistanceToTarget, 2));

    if(visionTarget.equals("Red Hub") || visionTarget.equals("Blue Hub")) {
      hoodAngleTarget = diagDistanceToTarget * (45 / 6.231);
    } 
    else {
      hoodAngleTarget = 45;
    }
    
    return hoodAngleTarget;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return autoChooser.getSelected();
    // return autonPreloadCommandGroup().withTimeout(20);
    return null;
  }
}

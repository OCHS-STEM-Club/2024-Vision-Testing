// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.commands.Climber.ClimberDownCommand;
import frc.robot.commands.Climber.ClimberDownOverrideCmd;
import frc.robot.commands.Climber.ClimberUpCommand;
import frc.robot.commands.Climber.ClimberUpOverrideCmd;
import frc.robot.commands.Intake.IntakeInCommand;
import frc.robot.commands.Intake.IntakeOutCommand;
import frc.robot.commands.Intake.IntakeOverrideCommand;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.commands.Shooter.ShooterShuttleCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));
 
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public CommandXboxController m_buttonBox = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private final SendableChooser<Command> autoChooser;


  //Commands
  ArmCommand m_manualArmUpCommand = new ArmCommand(m_armSubsystem, ArmConstants.kArmUpSpeed);
  ArmCommand m_manualArmDownCommand = new ArmCommand(m_armSubsystem, ArmConstants.kArmDownSpeed);
  ShooterCommand m_shooterCommand = new ShooterCommand(m_shooterSubsystem);
  IntakeInCommand m_intakeInCommand = new IntakeInCommand(m_intakeSubsystem);
  IntakeOverrideCommand m_intakeOverrideCommand = new IntakeOverrideCommand(m_intakeSubsystem);
  IntakeOutCommand m_intakeOutCommand = new IntakeOutCommand(m_intakeSubsystem);
  ClimberDownOverrideCmd m_climberDownOverrideCmd = new ClimberDownOverrideCmd(m_climberSubsystem);
  ClimberUpOverrideCmd m_climberUpOverrideCmd = new ClimberUpOverrideCmd(m_climberSubsystem);
  ClimberDownCommand m_climberDownCommand = new ClimberDownCommand(m_climberSubsystem);
  ClimberUpCommand m_climberUpCommand = new ClimberUpCommand(m_climberSubsystem);
  ShooterShuttleCmd m_shooterShuttleCommand = new ShooterShuttleCmd(m_shooterSubsystem);


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getRawAxis(4) * -1, OperatorConstants.RIGHT_X_DEADBAND));
      // () -> 0);

  Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRawAxis(2));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Intake
    NamedCommands.registerCommand("Intake in Override", Commands.runOnce(() -> m_intakeSubsystem.intakeSpeed(0.4)));
    NamedCommands.registerCommand("Intake Out", Commands.runOnce(() -> m_intakeSubsystem.intakeSpeed(-0.4)));
    NamedCommands.registerCommand("Intake in BB", new IntakeInCommand(m_intakeSubsystem));
    NamedCommands.registerCommand("Intake Off", Commands.runOnce(m_intakeSubsystem::intakeOff));

    // Shooter
    NamedCommands.registerCommand("Shooter On", Commands.runOnce(() -> m_shooterSubsystem.shooterOn(0.4)));
    NamedCommands.registerCommand("Shooter On 4 Piece", Commands.runOnce(() -> m_shooterSubsystem.shooterOn(0.45)));
    NamedCommands.registerCommand("Shooter Off", Commands.runOnce(m_shooterSubsystem::shooterOff));

    // Arm
    NamedCommands.registerCommand("Arm to Shooter 1st Piece Middle", Commands.runOnce(() -> m_armSubsystem.setReference(27)));
    NamedCommands.registerCommand("Arm to Shooter Shuttle", Commands.runOnce(() -> m_armSubsystem.setReference(27)));
    NamedCommands.registerCommand("Arm to Shooter 4 Piece", Commands.runOnce(() -> m_armSubsystem.setReference(31)));
    NamedCommands.registerCommand("Arm to Shooter 4 Piece 1st", Commands.runOnce(() -> m_armSubsystem.setReference(37)));
    NamedCommands.registerCommand("Arm to Shooter Sides", Commands.runOnce(() -> m_armSubsystem.setReference(7)));
    NamedCommands.registerCommand("Arm to Shooter Midlfield 2 piece", Commands.runOnce(() -> m_armSubsystem.setReference(23)));
    NamedCommands.registerCommand("Arm to Intake", Commands.runOnce(m_armSubsystem::intakeSetpoint));
    NamedCommands.registerCommand("Arm to Amp", Commands.runOnce(m_armSubsystem::ampSetpoint));
    NamedCommands.registerCommand("Arm to Shooter Subwoofer", Commands.runOnce(m_armSubsystem::shooterSetpoint));
    NamedCommands.registerCommand("Arm to Shooter Side Source 1st Piece", Commands.runOnce(() -> m_armSubsystem.setReference(26.5)));
    NamedCommands.registerCommand("Arm to Shooter Side Source 1st Piece Test", Commands.runOnce(() -> m_armSubsystem.setReference(23)));


    autoChooser = AutoBuilder.buildAutoChooser();
    // Configure the trigger bindings
    configureBindings();

    SmartDashboard.putData("Autos", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    if (DriverStation.isTest())
    {
      driverXbox.b().whileTrue(
        drivebase.sysIdDriveMotorCommand()
      );
      driverXbox.x().whileTrue(
        Commands.runOnce(drivebase::lock, drivebase).repeatedly()
      );
      driverXbox.y().whileTrue(
        drivebase.driveToDistanceCommand(1.0, 0.2)
      );
      driverXbox.start().onTrue(
        (Commands.runOnce(drivebase::zeroGyro))
      );
      driverXbox.back().whileTrue(
        drivebase.centerModulesCommand()
      );
      driverXbox.leftBumper().onTrue(
        Commands.none()
      );
      driverXbox.rightBumper().onTrue(
        Commands.none()
      );


      drivebase.setDefaultCommand(
          !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedAnglularVelocity);

    } else
    {
      driverXbox.a().onTrue(
        (Commands.runOnce(drivebase::zeroGyro))
      );
      // driverXbox.x().onTrue(
      //   Commands.runOnce(drivebase::addFakeVisionReading)
      // );
      // driverXbox.b().whileTrue(
      //     Commands.deferredProxy(() -> drivebase.driveToPose(
      //                                new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      //                           ));
      
      // driverXbox.leftBumper().whileTrue(
      //   Commands.runOnce(drivebase::lock, drivebase).repeatedly()
      // );
    driverXbox.a().onTrue(
        (Commands.runOnce(drivebase::zeroGyro))
      );

    driverXbox.y().whileTrue(
        drivebase.aimAtSpeaker(2)
      );
      
    driverXbox.rightTrigger().whileTrue(
      m_shooterCommand
    );

    driverXbox.leftTrigger().whileTrue(
      m_intakeInCommand
    );

    driverXbox.b().whileTrue(
      m_intakeOverrideCommand
    );

    driverXbox.leftBumper().whileTrue(
      m_intakeOutCommand
    );

    driverXbox.rightBumper().whileTrue(
      m_shooterShuttleCommand
    );

    // Button Box Configs
    m_buttonBox.button(1).whileTrue(
      m_manualArmDownCommand
    );

    m_buttonBox.button(3).whileTrue(
      m_manualArmUpCommand
    );

    m_buttonBox.button(4).whileTrue(
      Commands.runOnce(m_armSubsystem::intakeSetpoint)
    );

    m_buttonBox.button(6).whileTrue(
      Commands.runOnce(m_armSubsystem::shooterSetpoint)
    );
      
    m_buttonBox.button(5).whileTrue(
      Commands.runOnce(m_armSubsystem::ampSetpoint)
    );

    m_buttonBox.button(10).whileTrue(
      m_climberUpOverrideCmd
    );

    m_buttonBox.button(9).whileTrue(
      m_climberDownOverrideCmd
    );

    m_buttonBox.pov(0).whileTrue(
      m_climberUpCommand
    );

    m_buttonBox.pov(180).whileTrue(
      m_climberDownCommand
    );

      drivebase.setDefaultCommand(
          !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedAnglularVelocity);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}

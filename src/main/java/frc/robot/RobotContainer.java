// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// This class is where the bulk of the robot should be declared.  Since Command-based is a
// "declarative" paradigm, very little robot logic should actually be handled in the Robot
// periodic methods (other than the scheduler calls).  Instead, the structure of the robot
// (including subsystems, commands, and button mappings) should be declared here.
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  private final ShooterSubsystem m_shooterSubystem = new ShooterSubsystem();

  // New Choosing Option in SmartDashboard for Autos
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.k_driverController);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    // Makes the drive command the default command (good!)
    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

  // Trigger & Button Bindings!
  private void configureBindings() {

    // Intake
    new JoystickButton(m_driverController.getHID(), DriveConstants.k_intakeButton)
      .whileTrue(
        new InstantCommand(() -> m_intakeSubsystem.intake(), m_intakeSubsystem))
      .whileFalse(
        new InstantCommand(() -> m_intakeSubsystem.stop(), m_intakeSubsystem)
      );
    
    // Spin Up
    new Trigger(() -> m_driverController.getRawAxis(DriveConstants.k_spinUpTrigger) > 0.05)
      .whileTrue(
        new InstantCommand(() -> m_shooterSubystem.spinUp(), m_shooterSubystem).alongWith(
        new InstantCommand(() -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 1))))
      .whileFalse(
        new InstantCommand(() -> m_shooterSubystem.stop(), m_shooterSubystem).alongWith(
        new InstantCommand(() -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0)))
      );
    
    // Pew pew!
    new Trigger(() -> m_driverController.getRawAxis(DriveConstants.k_shootTrigger) > 0.05)
      .whileTrue(
        new InstantCommand(() -> m_shooterSubystem.shoot(), m_shooterSubystem))
      .whileFalse(
        new InstantCommand(() -> m_shooterSubystem.stop(), m_shooterSubystem)
      );
  }
  
  // Command that takes Xbox Controller Inputs and allows robot to drive
  // NOTE: getLeftY and getLeftX are opposite for a reason!!! It is correct!!
  Command driveFieldOrientedAngularVelocity = m_swerveSubsystem.driveCommand(
      () -> MathUtil.applyDeadband(m_driverController.getLeftY() * DriveConstants.k_driveSpeed, DriveConstants.k_driveDeadBand),
      () -> MathUtil.applyDeadband(m_driverController.getLeftX() * DriveConstants.k_driveSpeed, DriveConstants.k_driveDeadBand),
      () -> m_driverController.getRightX() * DriveConstants.k_turnRate);

  // Use this to pass the autonomous command to Robot.java
  // Returns the command to run in autonomous
  public Command getAutonomousCommand() {

    // The selected auto on SmartDashboard will be run in autonomous
    return m_chooser.getSelected(); 
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;

public final class Constants {

  // Constants for Controller IDs
  public static final class OperatorConstants {
    public static final int k_driverController = 0;
  }

  // Constants for Kraken Drivetrain!
  public static final class SwerveConstants {
    public static final double k_maxSpeed = Units.feetToMeters(18.9);
  }

  // Constants for controller input!
  public static final class DriveConstants {

    // YAGSL Swerve Stuff (Don't touch)
    public static final double k_driveDeadBand = 0.05;
    public static final double k_driveSpeed = -1;
    public static final double k_turnRate = -1;

    // Driver Controls 
    public final static int k_intakeButton = Button.kRightBumper.value; // Right Bump
    public final static int k_spinUpTrigger = Axis.kRightTrigger.value; // Right Trig
    public final static int k_shootTrigger = Axis.kLeftTrigger.value; // Left Trig
  }

  // Constants for Motor IDs
  public static final class MotorIDConstants {

    // Intake
    public static final int k_intakeBottomMotorID = 11;
    public static final int k_intakeTopMotorID = 12;

    // Indexer
    public static final int k_indexerBottomMotorID = 21;
    public static final int k_indexerTopMotorID =  22;

    // Shooter
    public static final int k_shooterIndexerMotorID = 31;
    public static final int k_shooterMotorID = 32;
  }

  // Constants for Intake
  public static final class IntakeConstants {
    public static final double k_intakeKrakenSpeed = 0.95;
  }

  // Constants for Indexer
  public static final class IndexerConstants {
    public static final double k_indexerKrakenSpeed = 0.95;
  }

  // Constants for Shooter
  public static final class ShooterConstants {
    public static final double k_shooterIndexerKrakenSpeed = 0.6;
    public static final double k_shooterKrakenSpeed = 0.95;
  }

  // Constants for Motors
  public static final class MotorConstants {
    public static final double k_rampRate = 0.40;
    public static final double k_closedMaxSpeed = 0.95;
    public static final int k_supplyCurrentLimit = 40;
  }

  // Constants for PID
  public static final class PIDConstants {
    public static final double k_intakekP = 0.0;
    public static final double k_intakekI = 0.0;
    public static final double k_intakekD = 0.0;
  }
}

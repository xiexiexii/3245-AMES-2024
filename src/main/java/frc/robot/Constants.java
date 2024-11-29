// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

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
    public static final double k_maxSpeed = Units.feetToMeters(5); // 18.9 is max
  }

  // Constants for controller input!
  public static final class DriveConstants {

    // YAGSL Swerve Stuff (Don't touch)
    public static final double k_driveDeadBand = 0.1;
    public static final double k_driveSpeed = -1;
    public static final double k_turnRate = -1;

    // Driver Controls 
    public final static int k_intakeButton = Button.kRightBumper.value; // Right Bump
    public final static int k_spinUpTrigger = Axis.kRightTrigger.value; // Right Trig
    public final static int k_shootTrigger = Axis.kLeftTrigger.value; // Left Trig
    public final static int k_zeroGyroButton = Button.kStart.value; // Start Button
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
    public static final double k_shooterIndexerKrakenSpeed = 0.1;
    public static final double k_shooterKrakenSpeed = 0.3;
  }

  // Constants for Motors
  public static final class MotorConstants {
    public static final double k_rampRate = 0.05;
    public static final double k_closedMaxSpeed = 0.8;
    public static final int k_supplyCurrentLimit = 40;
  }

  // Constants for PID
  public static final class MotorPIDConstants {
    public static final double k_intakekP = 0.0;
    public static final double k_intakekI = 0.0;
    public static final double k_intakekD = 0.0;
    public static final double k_intakekS = 0.0;
    public static final double k_intakekV = 0.0;
  }

  // Constants for Autonomous
  public static final class AutoConstants {
    public static final PIDConstants k_translationPID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants k_anglePID = new PIDConstants(0.4, 0, 0.01);
  }
}

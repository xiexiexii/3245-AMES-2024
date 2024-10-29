// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

  // Constants for controller IDs
  public static final class OperatorConstants {
    public static final int k_driverController = 0;
  }

  // Constants for Kraken Drivetrain!
  public static final class SwerveConstants {
    public static final double k_maxSpeed = Units.feetToMeters(18.9);
  }

  // Constants for controller input!
  public static final class DriveConstants {
    public static final double k_driveDeadBand = 0.05;
    public static final double k_driveSpeed = -1;
    public static final double k_turnRate = -1;
  }
}

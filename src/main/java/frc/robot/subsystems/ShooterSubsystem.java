// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.ShooterConstants;

// Shooter Subsystem Code yippee
public class ShooterSubsystem extends SubsystemBase {

  // Init Stuff
  private TalonFX m_shooterIndexerMotor;
  private TalonFX m_shooterMotor;

  // Other setup items
  public ShooterSubsystem() {
    
    // KRAKENS
    m_shooterIndexerMotor = new TalonFX(MotorIDConstants.k_shooterIndexerMotorID);
    m_shooterMotor = new TalonFX(MotorIDConstants.k_shooterMotorID);

  }

  // Runs once per scheduler run
  public void periodic() {}
  
  // Spin up command
  public void spinUp() {
    m_shooterMotor.set(ShooterConstants.k_shooterKrakenSpeed);
  }

  // Shoots yippeee
  public void shoot() {
    m_shooterIndexerMotor.set(ShooterConstants.k_shooterIndexerKrakenSpeed);
    m_shooterMotor.set(ShooterConstants.k_shooterKrakenSpeed);
  }

  // Stops motors
  public void stop() {
    m_shooterIndexerMotor.set(0);
    m_shooterMotor.set(0);
  }
}

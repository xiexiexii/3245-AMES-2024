// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.MotorIDConstants;

// Indexer Subsystem Code yippee
public class IndexerSubsystem extends SubsystemBase {

  // Init Stuff
  private TalonFX m_indexerBottomMotor;
  private TalonFX m_indexerTopMotor;

  // Other setup items
  public IndexerSubsystem() {
    
    // KRAKENS
    m_indexerBottomMotor = new TalonFX(MotorIDConstants.k_indexerBottomMotorID);
    m_indexerTopMotor = new TalonFX(MotorIDConstants.k_indexerTopMotorID);

  }

  // Runs once per scheduler run
  public void periodic() {}
  
  // Indexes yippeee
  public void index() {
    m_indexerBottomMotor.set(IndexerConstants.k_indexerKrakenSpeed);
    m_indexerTopMotor.set(-IndexerConstants.k_indexerKrakenSpeed);
  }

  // Stops motors
  public void stop() {
    m_indexerBottomMotor.set(0);
    m_indexerTopMotor.set(0);
  }
}

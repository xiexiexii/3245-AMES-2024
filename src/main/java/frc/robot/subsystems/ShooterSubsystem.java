// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorPIDConstants;
import frc.robot.Constants.ShooterConstants;

// Shooter Subsystem Code yippee
public class ShooterSubsystem extends SubsystemBase {

  // Init Stuff
  private TalonFX m_shooterIndexerMotor;
  private TalonFX m_shooterMotor;
  private TalonFXConfiguration krakenConfig;

  // Other setup items
  public ShooterSubsystem() {
    
    // KRAKENS
    m_shooterIndexerMotor = new TalonFX(MotorIDConstants.k_shooterIndexerMotorID);
    m_shooterMotor = new TalonFX(MotorIDConstants.k_shooterMotorID);

    // Init krakenConfig
    krakenConfig = new TalonFXConfiguration();

    // PID Stuff
    krakenConfig.Slot0.kP = MotorPIDConstants.k_intakekP;
    krakenConfig.Slot0.kI = MotorPIDConstants.k_intakekI;
    krakenConfig.Slot0.kD = MotorPIDConstants.k_intakekD;
    krakenConfig.Slot0.kS = MotorPIDConstants.k_intakekS;
    krakenConfig.Slot0.kV = MotorPIDConstants.k_intakekV;

    // Kraken Configs
    krakenConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = MotorConstants.k_rampRate;
    krakenConfig.MotorOutput.PeakForwardDutyCycle = MotorConstants.k_closedMaxSpeed;
    krakenConfig.MotorOutput.PeakReverseDutyCycle = -MotorConstants.k_closedMaxSpeed;
    krakenConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    krakenConfig.CurrentLimits.SupplyCurrentLimit = MotorConstants.k_supplyCurrentLimit;

    // Apply Configs, Inversion, Control requests
    m_shooterIndexerMotor.getConfigurator().apply(krakenConfig, 0.05);
    m_shooterMotor.getConfigurator().apply(krakenConfig, 0.05);

    m_shooterIndexerMotor.setInverted(true);
    m_shooterMotor.setInverted(false);
  }

  // Runs once per scheduler run
  public void periodic() {}
  
  // Spin up command
  public void spinUp() {
    m_shooterMotor.set(ShooterConstants.k_shooterKrakenSpeed);
  }

  // Spin up command
  public void indexUp() {
    m_shooterMotor.set(ShooterConstants.k_shooterIndexerKrakenSpeed);
  }

  // Shoots yippeee
  public void shoot() {
    m_shooterIndexerMotor.set(ShooterConstants.k_shooterIndexerKrakenSpeed);
  }

  // Stops motors
  public void stop() {
    m_shooterIndexerMotor.set(0);
    m_shooterMotor.set(0);
  }

  public void stopShoot() {
    m_shooterIndexerMotor.set(0);
  }
}

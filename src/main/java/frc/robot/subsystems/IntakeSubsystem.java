// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.PIDConstants;

// Intake Subsystem Code yippee
public class IntakeSubsystem extends SubsystemBase {

  // Init Stuff
  private TalonFX m_intakeBottomMotor;
  private TalonFX m_intakeTopMotor;
  private TalonFXConfiguration krakenConfig;
  private Slot0Configs slot0Configs;

  // Other setup items
  public IntakeSubsystem() {
    
    // Krakens!
    m_intakeBottomMotor = new TalonFX(MotorIDConstants.k_intakeBottomMotorID);
    m_intakeTopMotor = new TalonFX(MotorIDConstants.k_intakeTopMotorID);

    // PID Stuff
    slot0Configs = new Slot0Configs();
    slot0Configs.kP = PIDConstants.k_intakekP;
    slot0Configs.kI = PIDConstants.k_intakekI;
    slot0Configs.kD = PIDConstants.k_intakekD;

    // Kraken Configs
    krakenConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = MotorConstants.k_rampRate;
    krakenConfig.MotorOutput.PeakForwardDutyCycle = MotorConstants.k_closedMaxSpeed;
    krakenConfig.MotorOutput.PeakReverseDutyCycle = -MotorConstants.k_closedMaxSpeed;
    krakenConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    krakenConfig.CurrentLimits.SupplyCurrentLimit = MotorConstants.k_supplyCurrentLimit;

    // Apply Configs, Inversion, Control requests
    m_intakeBottomMotor.getConfigurator().apply(krakenConfig);
    m_intakeTopMotor.getConfigurator().apply(krakenConfig);
    m_intakeBottomMotor.getConfigurator().apply(slot0Configs, 0.05);

    m_intakeBottomMotor.setInverted(false);
    m_intakeTopMotor.setControl(new Follower(m_intakeBottomMotor.getDeviceID(), true));
  }

  // Runs once per scheduler run
  public void periodic() {}
  
  // Intakes yippeee
  public void intake() {
    m_intakeBottomMotor.set(IntakeConstants.k_intakeKrakenSpeed);
  }

  // Stops Motors
  public void stop() {
    m_intakeBottomMotor.set(0);
  }
}

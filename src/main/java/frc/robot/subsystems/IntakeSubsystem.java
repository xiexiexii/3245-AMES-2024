// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorIDConstants;

// Intake Subsystem Code yippee
public class IntakeSubsystem extends SubsystemBase {

  // Init Stuff
  private TalonFX m_intakeBottomMotor;
  private TalonFX m_intakeTopMotor;

  // Other setup items
  public IntakeSubsystem() {
    
    // KRAKENS
    m_intakeBottomMotor = new TalonFX(MotorIDConstants.k_intakeBottomMotorID);
    m_intakeTopMotor = new TalonFX(MotorIDConstants.k_intakeTopMotorID);

  }

  // Runs once per scheduler run
  public void periodic() {}
  
  // Intakes yippeee
  public void intake() {
    m_intakeBottomMotor.set(IntakeConstants.k_intakeKrakenSpeed);
    m_intakeTopMotor.set(-IntakeConstants.k_intakeKrakenSpeed);
  }

  // Stops Motors
  public void stop() {
    m_intakeBottomMotor.set(0);
    m_intakeTopMotor.set(0);
  }
}

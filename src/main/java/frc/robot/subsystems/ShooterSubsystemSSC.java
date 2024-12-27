// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorPIDConstants;
import frc.robot.Constants.ShooterConstants;

// Shooter Subsystem Code yippee
public class ShooterSubsystemSSC extends SubsystemBase {

  // Init Stuff
  private TalonFX m_shooterIndexerMotor;
  private TalonFX m_shooterMotor;
  private TalonFXConfiguration krakenConfig;

  // The plant (subsystem) holds a state-space model of our flywheel. This system has the following properties:
  // States: [velocity], in radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [velocity], in radians per second.
  private final LinearSystem<N1, N1, N1> m_shooterPlant = LinearSystemId.createFlywheelSystem(

    // Number of Motors in Gearbox (Important: Not motor ID!!)
    DCMotor.getKrakenX60(1), 

    // Moment of Inertia of Flywheel in kg * m^2 (Get from CAD or do the math; Adjust me!!)
    ShooterConstants.kFlywheelMOI, 

    // Gearing of Flywheel (Get from CAD or do the math; Adjust me!!)
    // Reduction between motors and encoder, as output over input. If the flywheel 
    // spins slower than the motors, this number should be greater than one.
    ShooterConstants.kFlywheelGearing 
  );

  // Kalman filters are used to filter velocity measurements using our state-space model to generate a state estimate
  // The Kalman observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> m_observer = new KalmanFilter<>(

    // Natural Number representing states of a system
    Nat.N1(),

    // Natural number representing outputs of a system
    Nat.N1(), 

    // The Linear System we made above
    m_shooterPlant, 

    // StdDev for of states (How accurate we think our model is)
    VecBuilder.fill(3.0), 

    // StdDev for measurements (How accurate we think our encoder data is)
    VecBuilder.fill(0.01), 

    // Nominal time between loops. 0.020s (20 ms) for a typical robot, 
    // but can be lower if using notifiers.
    0.020 
  );

  // A Linear Quadratic Regulator (LQR) is a feedback control scheme which seeks
  // to operate a system in a “most optimal” or “lowest cost” manner
  // The LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N1, N1, N1> m_controller = new LinearQuadraticRegulator<>(

    // Our little system again!
    m_shooterPlant, 

    // qelms - Velocity error tolerance, in radians per second. Decrease
    // to more heavily penalize state excursion, or make the controller behave more aggressively.
    VecBuilder.fill(8.0), 
    
    // relms - Controls effort (voltage) tolerance. Decrease this to more
    // heavily penalize control effort, or make the controller less aggressive. 12 is a good
    // starting point because that is the (approximate) maximum voltage of a battery.
    VecBuilder.fill(12.0), 
    
    // Nominal time between loops. 0.020s (20 ms) for a typical robot, 
    // but can be lower if using notifiers.
    0.020
  );

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  // Do you want to manage three things or one? One, right? If you like to live life the hard way,
  // go write your own code <3
  private final LinearSystemLoop<N1, N1, N1> m_loop = new LinearSystemLoop<>(

    // The Plant ("Subsystem" being moderated)
    m_shooterPlant, 

    // The Linear Quadratic Regulator (Cost Reducer)
    m_controller, 

    // The Kalman Filter (Noise Rejecter)
    m_observer, 

    // Max voltage that can be applied (12V bc we're FRC)
    12.0, 

    // Nominal time between loops. 0.020s (20 ms) for a typical robot, 
    // but can be lower if using notifiers.
    0.020
  );

  // Other setup items
  public ShooterSubsystemSSC() {
    
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
  public void periodic() {

    // Correct our Kalman filter's state vector estimate with encoder data.
    // Because we have Krakens, we can use its built-in encoder's velocity value!
    m_loop.correct(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(m_shooterMotor.getVelocity().getValueAsDouble() * 60)));

    // Update our LQR to generate new voltage commands and use the voltages 
    // to predict the next state with our Kalman filter.
    m_loop.predict(0.020);

    // Send new calculated voltage to motors. voltage = duty cycle * battery voltage,
    // so duty cycle = voltage / battery voltage (getU method gets control input)
    double nextVoltage = m_loop.getU(0);
    m_shooterMotor.setVoltage(nextVoltage);

  }
  
  // Spin up command
  public void spinUp() {

    // Sets the target speed of our flywheel. This is similar to 
    // setting the setpoint of a PID controller. The next reference
    // "R" is set to the desired speed in rad/s
    m_loop.setNextR(VecBuilder.fill(ShooterConstants.kSpinupRadPerSec));

  }

  // Index up command
  public void indexUp() {
    m_shooterMotor.set(ShooterConstants.k_shooterIndexerKrakenSpeed);
  }

  // Shoots yippeee
  public void shoot() {
    m_shooterIndexerMotor.set(ShooterConstants.k_shooterIndexerKrakenSpeed);
  }

  // Stops motors
  public void stop() {

    // Stop Index Kraken
    m_shooterIndexerMotor.set(0);

    // Sets next reference "R" to zero to stop
    m_loop.setNextR(VecBuilder.fill(0.0));

  }

  public void stopShoot() {
    m_shooterIndexerMotor.set(0);
  }
}

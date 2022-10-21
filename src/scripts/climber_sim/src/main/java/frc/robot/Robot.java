package frc.robot;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** This is a sample program to demonstrate the use of elevator simulation with existing code. */
public class Robot extends TimedRobot {
  private static final int kMotorPort = 0;
  private static final int kEncoderAChannel = 0;
  private static final int kEncoderBChannel = 1;
  private static final int kJoystickPort = 0;

  private static final double kElevatorGearing = 15.6;
  private static final double kElevatorDrumRadius = Units.inchesToMeters(0.625);
  private static final double kCarriageMass = 24.0; // kg

  private static final double kMinElevatorHeight = Units.inchesToMeters(2);
  private static final double kMaxElevatorHeight = Units.inchesToMeters(50);

  // distance per pulse = (distance per revolution) / (pulses per revolution)
  //  = (Pi * D) / ppr
  private static final double kElevatorEncoderDistPerPulse =
      2.0 * Math.PI * kElevatorDrumRadius / 4096;

  private final DCMotor m_elevatorGearbox = DCMotor.getFalcon500(4);

  // Standard classes for controlling our elevator
  //  private final PIDController m_controller = new PIDController(200, 0.2, 20);
  private final PIDController m_controller = new PIDController(25, 0.1, 4);
  private final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);
  private final PWMTalonFX m_motor = new PWMTalonFX(kMotorPort);
  private final Joystick m_joystick = new Joystick(kJoystickPort);

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          kElevatorGearing,
          kCarriageMass,
          kElevatorDrumRadius,
          kMinElevatorHeight,
          kMaxElevatorHeight,
          VecBuilder.fill(0.0));
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Elevator", Units.metersToInches(m_elevatorSim.getPositionMeters()), 90));

  @Override
  public void robotInit() {
    m_controller.setIntegratorRange(-1, 1);
    m_controller.setTolerance(0.005);
    m_encoder.setDistancePerPulse(kElevatorEncoderDistPerPulse);
    // Publish Mechanism2d to SmartDashboard
    SmartDashboard.putData("Elevator Sim", m_mech2d);
  }

  @Override
  public void simulationPeriodic() {
    if (m_controller.atSetpoint()) {
      m_controller.reset();
      System.out.println("At Setpoint");
    } else {
      System.out.println();
    }
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    SmartDashboard.putNumber("Plant", m_elevatorSim.getPositionMeters());
    SmartDashboard.putNumber("Reference", Units.inchesToMeters(40));

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

    // Update elevator visualization with simulated position
    m_elevatorMech2d.setLength(Units.metersToInches(m_elevatorSim.getPositionMeters()));
  }

  @Override
  public void teleopPeriodic() {
    double pidOutput =
        m_controller.calculate(m_encoder.getDistance(), Units.inchesToMeters(40))
            + 0.1; // arbitrary feedforward accounts for gravity
    m_motor.setVoltage(pidOutput);
    SmartDashboard.putNumber("PID Response", pidOutput);
  }

  @Override
  public void disabledInit() {
    // This just makes sure that our simulation code knows that the motor's off.
    m_motor.set(0.0);
  }
}

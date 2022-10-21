package com.peninsula.frc2022.util.control;

import com.peninsula.frc2022.config.SwerveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveOutputs {

  private final double[] outputsVoltage = new double[4];
  private final double[] outputsRadians = new double[4];
  private SwerveModuleState[] states = new SwerveModuleState[4];
  private Rotation2d rotation2d = new Rotation2d(0);

  private boolean idle = false;

  public SwerveOutputs() {
    for (int i = 0; i < 4; i++) {
      outputsVoltage[i] = 0;
      outputsRadians[i] = 0;
    }
  }

  public SwerveOutputs(SwerveModuleState[] moduleStates) {
    setOutputs(moduleStates);
  }

  public void setOutputs(SwerveModuleState[] moduleStates) {
    for (int i = 0; i < 4; i++) {
      outputsVoltage[i] =
          moduleStates[i].speedMetersPerSecond
              / SwerveConstants.kMaxVelocityMetersPerSecond
              * SwerveConstants.kMaxVoltage;
      outputsRadians[i] = moduleStates[i].angle.getRadians();
    }
    states = moduleStates;
  }

  public void setOutputs(SwerveModuleState[] moduleStates, Rotation2d rotation2d) {
    for (int i = 0; i < 4; i++) {
      outputsVoltage[i] =
          moduleStates[i].speedMetersPerSecond
              / SwerveConstants.kMaxVelocityMetersPerSecond
              * SwerveConstants.kMaxVoltage;
      outputsRadians[i] = moduleStates[i].angle.getRadians();
    }
    states = moduleStates;
    this.rotation2d = rotation2d;
  }

  public Rotation2d getRotation2d() {
    return rotation2d;
  }

  public SwerveModuleState[] getStates() {
    return states;
  }

  public void setVoltages(double[] voltages) {
    System.arraycopy(voltages, 0, outputsVoltage, 0, 4);
  }

  public void setAngles(double[] radians) {
    System.arraycopy(radians, 0, outputsRadians, 0, 4);
  }

  public double[] steerAngles() {
    return outputsRadians;
  }

  public double[] voltages() {
    return outputsVoltage;
  }

  public boolean isIdle() {
    return idle;
  }

  public void setIdle(boolean idle) {
    this.idle = idle;
  }
}

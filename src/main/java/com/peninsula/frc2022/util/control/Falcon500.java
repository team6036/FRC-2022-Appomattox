package com.peninsula.frc2022.util.control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Falcon500 extends WPI_TalonFX {

  public Falcon500(int deviceNumber) {
    super(deviceNumber);
  }

  public void setOutputs(ControllerOutput outputs) {

    ControllerOutput.Mode mode = outputs.getControlMode();
    double demand = outputs.getArbitraryDemand();
    Gains gains = outputs.getGains();
    double reference = outputs.getReference();

    if (gains != null) {
      config_kP(0, gains.p);
      config_kI(0, gains.i);
      config_kD(0, gains.d);
      config_kF(0, gains.f);
    }

    switch (mode) {
      case VELOCITY:
      case PROFILED_VELOCITY:
        set(ControlMode.Velocity, demand, DemandType.ArbitraryFeedForward, reference);
        break;
      case PERCENT_OUTPUT:
        set(ControlMode.PercentOutput, demand, DemandType.ArbitraryFeedForward, reference);
        break;
      case POSITION:
      case PROFILED_POSITION:
        set(ControlMode.Position, demand, DemandType.ArbitraryFeedForward, reference);
    }
  }
}

package com.peninsula.frc2022.config;

import com.peninsula.frc2022.util.control.Gains;
import com.peninsula.frc2022.util.interpolation.InterpolatingDouble;
import com.peninsula.frc2022.util.interpolation.InterpolatingTreeMap;

public class IntakeConstants {

  public static double maxRollerPo = 0.8;
  public static double ejectRollerPo = -0.3;

  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> dtVel2IntakePo =
      new InterpolatingTreeMap<>();

  static {
    dtVel2IntakePo.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.45));
    dtVel2IntakePo.put(new InterpolatingDouble(2.0), new InterpolatingDouble(0.55));
    dtVel2IntakePo.put(new InterpolatingDouble(3.0), new InterpolatingDouble(0.65));
  }

  // Intake Arm
  public static double armUpPos = 15000;
  public static double armDownPos = -1000;

  public static double c = 0.08;

  public static Gains intakeArmGainsHard = new Gains(0.025, 0, 0, 0, 0, 0);
  public static Gains intakeArmGainsLight = new Gains(0.03, 0, 0, 0, 0, 0);
}

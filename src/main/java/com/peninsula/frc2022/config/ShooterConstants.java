package com.peninsula.frc2022.config;

import com.peninsula.frc2022.util.control.Gains;
import com.peninsula.frc2022.util.interpolation.InterpolatingDouble;
import com.peninsula.frc2022.util.interpolation.InterpolatingTreeMap;

public class ShooterConstants {

  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> flywheelDist2Vel =
      new InterpolatingTreeMap<>();
  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> hoodwheelDist2Vel =
      new InterpolatingTreeMap<>();

  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shotDist2Time =
      new InterpolatingTreeMap<>();

  static {
    flywheelDist2Vel.put(new InterpolatingDouble(0.5), new InterpolatingDouble(2900.0));
    flywheelDist2Vel.put(new InterpolatingDouble(1.0), new InterpolatingDouble(2000.0));
    flywheelDist2Vel.put(new InterpolatingDouble(2.0), new InterpolatingDouble(1900.0));
    flywheelDist2Vel.put(new InterpolatingDouble(3.0), new InterpolatingDouble(1800.0));

    flywheelDist2Vel.put(new InterpolatingDouble(3.5), new InterpolatingDouble(2000.0));
    flywheelDist2Vel.put(new InterpolatingDouble(4.0), new InterpolatingDouble(2200.0));
    flywheelDist2Vel.put(new InterpolatingDouble(5.0), new InterpolatingDouble(2400.0));

    hoodwheelDist2Vel.put(new InterpolatingDouble(0.5), new InterpolatingDouble(500.0));
    hoodwheelDist2Vel.put(new InterpolatingDouble(1.0), new InterpolatingDouble(1100.0));
    hoodwheelDist2Vel.put(new InterpolatingDouble(2.0), new InterpolatingDouble(1800.0));
    hoodwheelDist2Vel.put(new InterpolatingDouble(3.0), new InterpolatingDouble(3250.0));
    hoodwheelDist2Vel.put(new InterpolatingDouble(3.5), new InterpolatingDouble(3250.0));
    hoodwheelDist2Vel.put(new InterpolatingDouble(4.0), new InterpolatingDouble(3300.0));
    hoodwheelDist2Vel.put(new InterpolatingDouble(5.0), new InterpolatingDouble(3850.0));

    shotDist2Time.put(new InterpolatingDouble(1.0), new InterpolatingDouble(0.7));
    shotDist2Time.put(new InterpolatingDouble(2.0), new InterpolatingDouble(0.81));
    shotDist2Time.put(new InterpolatingDouble(3.0), new InterpolatingDouble(0.94));
    shotDist2Time.put(new InterpolatingDouble(4.0), new InterpolatingDouble(0.94));
    shotDist2Time.put(new InterpolatingDouble(5.0), new InterpolatingDouble(1.01));
    shotDist2Time.put(new InterpolatingDouble(6.0), new InterpolatingDouble(1.05));
    shotDist2Time.put(new InterpolatingDouble(7.0), new InterpolatingDouble(1.09));
  }

  public static Gains flywheelGains = new Gains(0.22, 0.0, 0.00002, 0.037, 0, 0); // 0.14 p
  public static Gains hoodwheelGains = new Gains(0.15, 0.0, 0.00, 0.0, 0, 0);
}

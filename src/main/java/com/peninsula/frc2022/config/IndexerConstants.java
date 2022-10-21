package com.peninsula.frc2022.config;

import com.peninsula.frc2022.util.control.Gains;

public class IndexerConstants {

  public static double intakeIndexerPo = 3.3, slightPo = 0.1, intakeIndexerVel = 100;
  public static Gains intakeIndexerGains = new Gains(0.1, 0, 0, 0, 0, 0);
  public static Gains kickerPosGains = new Gains(-0.000055, 0, 0.0000000002, 0, 0, 0);
}

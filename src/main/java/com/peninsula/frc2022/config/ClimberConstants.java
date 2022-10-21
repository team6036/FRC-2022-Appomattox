package com.peninsula.frc2022.config;

import com.peninsula.frc2022.util.control.Gains;

public class ClimberConstants {

  public static final double atPoseEpsilon = 16000;

  public static final double pastHighExtendDegrees = 30;
  public static final double contactHighDegrees = 37;
  public static final double traversalExtendDegrees = 23.2;
  public static final double contactTraversalDegrees = 22.6;

  public static final double rightDownTicks = 800;
  public static final double leftDownTicks = 800;
  public static final double rightPartialTicks = 230000;
  public static final double leftPartialTicks = 180000;
  public static final double rightExtendTicks = 236000;
  public static final double leftExtendTicks = 231000;

  public static final Gains pullerGains = new Gains(0.75, 0, 0, 0.079, 0, 0);

  public static final int accelerationTicksPer100MsPerSec = 50000;
  public static final int velocityTicksPer100ms = 34000;
}

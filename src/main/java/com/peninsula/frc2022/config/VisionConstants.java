package com.peninsula.frc2022.config;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

@SuppressWarnings("java:S1104")
public class VisionConstants {

  public static double camHeightM = 0.76;
  public static double targetheightM = 2.6416;
  public static double targetwidthM = 0.61;
  public static double cameraPitchRad = 35.5 * Math.PI / 180.0;
  public static Pose2d target = new Pose2d(8.23, 4.11, new Rotation2d(0));
}

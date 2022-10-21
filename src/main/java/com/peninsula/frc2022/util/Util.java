package com.peninsula.frc2022.util;

/** This class holds a bunch of static methods and variables needed for mathematics */
public class Util {

  private Util() {}

  /**
   * Neutralizes a value within a dead band
   *
   * @param value Value to control dead band
   * @param deadBand Value of dead band
   * @return 0 if within dead band, otherwise values
   */
  public static double handleDeadBand(double value, double deadBand) {
    return (Math.abs(value) > Math.abs(deadBand)) ? value : 0.0;
  }

  public static double clamp(double value, double minimum, double maximum) {
    return Math.min(maximum, Math.max(minimum, value));
  }

  public static String classToJsonName(Class<?> clazz) {
    if (clazz.isAnonymousClass()) {
      return "anonymous" + clazz.getSuperclass().getSimpleName();
    }
    String className = clazz.getSimpleName();
    // Make first character lowercase to match JSON conventions
    return Character.toLowerCase(className.charAt(0)) + className.substring(1);
  }
}

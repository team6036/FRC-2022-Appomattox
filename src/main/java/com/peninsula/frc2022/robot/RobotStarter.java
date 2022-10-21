package com.peninsula.frc2022.robot;

import edu.wpi.first.wpilibj.RobotBase;
import java.lang.reflect.Field;

public final class RobotStarter {

  static {
    try {
      Class<?> loggerClass = Class.forName("jdk.internal.module.IllegalAccessLogger");
      Field loggerField = loggerClass.getDeclaredField("logger");
      Class<?> unsafeClass = Class.forName("sun.misc.Unsafe");
      Field unsafeField = unsafeClass.getDeclaredField("theUnsafe");
      unsafeField.setAccessible(true);
      Object unsafe = unsafeField.get(null);
      var offset =
          (Long)
              unsafeClass.getMethod("staticFieldOffset", Field.class).invoke(unsafe, loggerField);
      unsafeClass
          .getMethod("putObjectVolatile", Object.class, long.class, Object.class)
          .invoke(unsafe, loggerClass, offset, null);
    } catch (Exception exception) {
    }
  }

  private RobotStarter() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}

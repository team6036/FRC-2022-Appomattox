package com.peninsula.frc2022.auto;

import com.peninsula.frc2022.behavior.RoutineBase;
import com.peninsula.frc2022.util.Util;

public interface AutoBase {

  RoutineBase getRoutine();

  default String getName() {
    return Util.classToJsonName(getClass());
  }
}

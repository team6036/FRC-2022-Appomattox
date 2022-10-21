package com.peninsula.frc2022.auto.climb;

import com.peninsula.frc2022.auto.AutoBase;
import com.peninsula.frc2022.behavior.RoutineBase;
import com.peninsula.frc2022.behavior.SequentialRoutine;
import com.peninsula.frc2022.behavior.routines.superstructure.climb.*;

public class AutoMid implements AutoBase {
  @Override
  public RoutineBase getRoutine() {
    var pullRightAndExtendLeftPartial = new RightStretchRoutine(0.2);

    return new SequentialRoutine(pullRightAndExtendLeftPartial);
  }
}

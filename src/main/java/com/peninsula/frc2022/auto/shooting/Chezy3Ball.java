package com.peninsula.frc2022.auto.shooting;

import com.pathplanner.lib.PathPlanner;
import com.peninsula.frc2022.auto.AutoBase;
import com.peninsula.frc2022.behavior.ParallelRaceRoutine;
import com.peninsula.frc2022.behavior.ParallelRoutine;
import com.peninsula.frc2022.behavior.RoutineBase;
import com.peninsula.frc2022.behavior.SequentialRoutine;
import com.peninsula.frc2022.behavior.routines.drive.DrivePathRoutine;
import com.peninsula.frc2022.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2022.behavior.routines.superstructure.*;

public class Chezy3Ball implements AutoBase {
  @Override
  public RoutineBase getRoutine() {

    var setInitialOdometry = new DriveSetOdometryRoutine(10.4, 3.04, 136.00);

    var goBack =
        new ParallelRaceRoutine(
            new ShooterOdometryRoutine(10.0),
            new DrivePathRoutine(PathPlanner.loadPath("MoveBackClimber", 1.5, 1.5)),
            new IntakeBallRoutine(10.0),
            new IntakeDownRoutine(10.0));

    var shoot3 =
        new ParallelRoutine(
            new IntakeBallRoutine(0.7),
            new IndexerFeedRoutine(0.7),
            new ShooterOdometryRoutine(0.7));

    return new SequentialRoutine(setInitialOdometry, goBack, shoot3, new IntakeUpRoutine(1.0));
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}

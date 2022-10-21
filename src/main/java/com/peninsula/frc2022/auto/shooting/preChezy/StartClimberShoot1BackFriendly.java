package com.peninsula.frc2022.auto.shooting.preChezy;

import com.pathplanner.lib.PathPlanner;
import com.peninsula.frc2022.auto.AutoBase;
import com.peninsula.frc2022.behavior.ParallelRaceRoutine;
import com.peninsula.frc2022.behavior.RoutineBase;
import com.peninsula.frc2022.behavior.SequentialRoutine;
import com.peninsula.frc2022.behavior.routines.TimedRoutine;
import com.peninsula.frc2022.behavior.routines.drive.DrivePathRoutine;
import com.peninsula.frc2022.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2022.behavior.routines.superstructure.*;

public class StartClimberShoot1BackFriendly implements AutoBase {

  @Override
  public RoutineBase getRoutine() {

    var setInitialOdometry = new DriveSetOdometryRoutine(10.4, 3.04, 136.00);

    var goBack =
        new ParallelRaceRoutine(
            new ShooterVisionRoutine(10.0),
            new DrivePathRoutine(PathPlanner.loadPath("MoveBackClimber", 3, 2)),
            new IntakeBallRoutine(10.0));

    var shootIntoHub =
        new ParallelRaceRoutine(
            new ShooterVisionRoutine(0.5), new IntakeBallRoutine(0.5), new IndexerFeedRoutine(0.5));

    var goBack2 =
        new ParallelRaceRoutine(
            new DrivePathRoutine(PathPlanner.loadPath("Bad", 3, 2)),
            new IntakeBallRoutine(10.0),
            new IndexerRoutine(10.0));

    var spit =
        new ParallelRaceRoutine(
            new IndexerFeedRoutine(2.0),
            new SequentialRoutine(new TimedRoutine(1.0), new ShooterSpitRoutine(1.0)));

    return new SequentialRoutine(setInitialOdometry, goBack, shootIntoHub, goBack2, spit);
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}

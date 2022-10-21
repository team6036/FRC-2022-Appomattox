package com.peninsula.frc2022.auto.shooting.preChezy;

import com.pathplanner.lib.PathPlanner;
import com.peninsula.frc2022.auto.AutoBase;
import com.peninsula.frc2022.behavior.ParallelRaceRoutine;
import com.peninsula.frc2022.behavior.ParallelRoutine;
import com.peninsula.frc2022.behavior.RoutineBase;
import com.peninsula.frc2022.behavior.SequentialRoutine;
import com.peninsula.frc2022.behavior.routines.drive.AlignRoutine;
import com.peninsula.frc2022.behavior.routines.drive.DrivePathRoutine;
import com.peninsula.frc2022.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2022.behavior.routines.superstructure.IndexerRoutine;
import com.peninsula.frc2022.behavior.routines.superstructure.IntakeBallRoutine;
import com.peninsula.frc2022.behavior.routines.superstructure.ShooterVisionRoutine;

public class StartIntakeShoot2FriendlyAlign implements AutoBase {

  @Override
  public RoutineBase getRoutine() {
    //
    var driveSetOdometry = new DriveSetOdometryRoutine(6.5, 3, 0);

    var moveBackAndIntake =
        new ParallelRaceRoutine(
            new IntakeBallRoutine(10.0),
            new DrivePathRoutine(PathPlanner.loadPath("BackFree", 3, 2)));

    var alignAndShoot =
        new ParallelRoutine(
            new ShooterVisionRoutine(3.0),
            new IntakeBallRoutine(3.0),
            new SequentialRoutine(new AlignRoutine(2.0), new IndexerRoutine(1.0)));

    return new SequentialRoutine(driveSetOdometry, moveBackAndIntake, alignAndShoot);
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}

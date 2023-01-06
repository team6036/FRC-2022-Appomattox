package com.peninsula.frc2022.behavior.routines.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.peninsula.frc2022.behavior.TimeoutRoutineBase;
import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.ReadOnly;
import com.peninsula.frc2022.robot.RobotState;
import com.peninsula.frc2022.subsystems.SubsystemBase;
import java.util.*;

/** Follows a pathplanner trajectory */
public class DrivePathRoutine extends TimeoutRoutineBase {

  private PathPlannerTrajectory mPathPlannerTrajectory;
  private boolean start = false;

  public DrivePathRoutine(PathPlannerTrajectory trajectory) {
    mPathPlannerTrajectory = trajectory;
    mTimeout = mPathPlannerTrajectory.getTotalTimeSeconds() * 1.05;
  }

  @Override
  public void start(Commands commands, @ReadOnly RobotState state) {
    // Required to start the timeout timer
    super.start(commands, state);
  }

  @Override
  public Set<SubsystemBase> getRequiredSubsystems() {
    return Set.of(mDrive);
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    if (!start) {
      commands.setDriveFollowPath(mPathPlannerTrajectory);
      state.currentTrajectory = mPathPlannerTrajectory;
      start = true;
    }
  }

  @Override
  public boolean checkIfFinishedEarly(@ReadOnly RobotState state) {
    // TODO: possibly implement to see if we are within a tolerance of the end early
    return false;
  }
}

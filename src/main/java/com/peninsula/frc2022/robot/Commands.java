package com.peninsula.frc2022.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.peninsula.frc2022.behavior.RoutineBase;
import com.peninsula.frc2022.subsystems.*;
import com.peninsula.frc2022.subsystems.controllers.climb.AutoClimbController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.List;

/** Commands represent what we want the robot to be doing. */
@SuppressWarnings("java:S1104")
public class Commands {

  /* Routines */
  public List<RoutineBase> routinesWanted = new ArrayList<>();
  public boolean shouldClearCurrentRoutines;
  /* Swerve */
  public Swerve.State swerveWanted;
  public boolean boostWanted;
  public boolean robotCentricWanted;
  public double angleWanted = -88;

  /* Intake */
  public Intake.State intakeWanted;
  public IntakeArm.State intakeArmWanted;

  /* Indexer */
  public Indexer.State indexerWanted;

  /* Shooter */
  private Shooter.State shooterWanted;
  private double shooterWantedRPM;
  private double hoodWantedRPM;

  /* Climber */
  public Climber.ClimbState climbWantedState = Climber.ClimbState.AUTO;

  public double rightTicksClimbWanted;
  public double leftTicksClimbWanted;

  public boolean zeroRoll = false;

  public AutoClimbController.RightArmState rightStateClimbWanted =
      AutoClimbController.RightArmState.DOWN;
  public AutoClimbController.LeftArmState leftStateClimbWanted =
      AutoClimbController.LeftArmState.DOWN;

  /* Vision */
  public Vision.State visionWanted;

  /* Lighting */
  public Lighting.State lightingWantedState;

  /* Auto */
  public PathPlannerTrajectory wantedPathPlannerTrajectory;
  public Pose2d driveWantedOdometryPose;
  public Rotation2d driveWantedOdometryPoseRotation;

  public void addWantedRoutines(RoutineBase... wantedRoutines) {
    for (RoutineBase wantedRoutine : wantedRoutines) {
      addWantedRoutine(wantedRoutine);
    }
  }

  public void addWantedRoutine(RoutineBase wantedRoutine) {
    routinesWanted.add(wantedRoutine);
  }

  /* Drive */
  @Override
  public String toString() {
    var log = new StringBuilder();
    log.append("Wanted routines: ");
    for (RoutineBase routine : routinesWanted) {
      log.append(routine).append(" ");
    }
    return log.append("\n").toString();
  }

  public void setShooterRPM(double wantedRPM) {
    shooterWanted = Shooter.State.VELOCITY;
    shooterWantedRPM = wantedRPM;
  }

  public double getHoodWantedRPM() {
    return hoodWantedRPM;
  }

  public void setShooterIdle() {
    shooterWanted = Shooter.State.OFF;
    shooterWantedRPM = 0;
  }

  public void setShooterVision() {
    shooterWanted = Shooter.State.VISION;
  }

  public void setShooterDriveBy() {
    shooterWanted = Shooter.State.SHOOT_ON_MOVE;
    swerveWanted = Swerve.State.SHOOT_ON_MOVE;
  }

  public void setShooterOnlyDriveBy() {
    shooterWanted = Shooter.State.SHOOT_ON_MOVE;
  }

  public Shooter.State getShooterWanted() {
    return shooterWanted;
  }

  public double getShooterWantedRPM() {
    return shooterWantedRPM;
  }

  public PathPlannerTrajectory getDriveWantedPathPlannerTrajectory() {
    return wantedPathPlannerTrajectory;
  }

  public void setDriveFollowPath(PathPlannerTrajectory pathPlannerTrajectory) {
    wantedPathPlannerTrajectory = pathPlannerTrajectory;
    swerveWanted = Swerve.State.AUTO;
  }
}

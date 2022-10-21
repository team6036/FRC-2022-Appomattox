package com.peninsula.frc2022.robot;

import com.peninsula.frc2022.auto.climb.AutoHigh;
import com.peninsula.frc2022.auto.climb.AutoMid;
import com.peninsula.frc2022.auto.climb.AutoTraversal;
import com.peninsula.frc2022.auto.shooting.Chezy3Ball;
import com.peninsula.frc2022.auto.shooting.Chezy3Ball2Rude;
import com.peninsula.frc2022.auto.shooting.Chezy6Ball;
import com.peninsula.frc2022.auto.shooting.ChezyMidBack;
import com.peninsula.frc2022.behavior.RoutineBase;
import com.peninsula.frc2022.behavior.RoutineManager;
import com.peninsula.frc2022.subsystems.*;
import com.peninsula.frc2022.subsystems.SubsystemBase;
import com.peninsula.frc2022.util.LoopOverrunDebugger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.*;

@SuppressWarnings("java:S1104")
public class Robot extends TimedRobot {

  public static final double kPeriod = 0.02;

  RoutineBase Auto_3_2 = new Chezy3Ball2Rude().getRoutine();
  RoutineBase Auto_3 = new Chezy3Ball().getRoutine();
  RoutineBase Auto_Mid = new ChezyMidBack().getRoutine();
  RoutineBase Auto_6 = new Chezy6Ball().getRoutine();

  SendableChooser<RoutineBase> autoChooser = new SendableChooser<>();
  public static SendableChooser<RoutineBase> climbChooser = new SendableChooser<>();

  private final RobotState mRobotState = new RobotState();
  private final Control mOperatorInterface = new Control();
  private final RoutineManager mRoutineManager = new RoutineManager();
  private final HardwareReader mHardwareReader = new HardwareReader();
  private final HardwareWriter mHardwareWriter = new HardwareWriter();
  private final Commands mCommands = new Commands();

  /* Subsystems */
  private final Swerve mSwerve = Swerve.getInstance();
  private final Intake mIntake = Intake.getInstance();
  private final IntakeArm mIntakeArm = IntakeArm.getInstance();
  private final Indexer mIndexer = Indexer.getInstance();
  private final Shooter mShooter = Shooter.getInstance();
  private final Climber mClimber = Climber.getInstance();
  private final Vision mVision = Vision.getInstance();
  private final Lighting mLighting = Lighting.getInstance();

  private Set<SubsystemBase> mSubsystems =
      Set.of(mSwerve, mIndexer, mShooter, mIntake, mVision, mLighting, mClimber, mIntakeArm);

  public static final LoopOverrunDebugger sLoopDebugger =
      new LoopOverrunDebugger("teleop", kPeriod);

  public Robot() {
    super(kPeriod);
  }

  @Override
  public void robotInit() {
    mHardwareWriter.configureHardware(mSubsystems);
    autoChooser.setDefaultOption("6 B", Auto_6);
    autoChooser.addOption("3 B", Auto_3);
    autoChooser.addOption("3 B 2 R", Auto_3_2);
    autoChooser.addOption("2 B", Auto_Mid);
    tab.add(autoChooser);

    climbChooser.setDefaultOption("Traversal", new AutoTraversal().getRoutine());
    climbChooser.addOption("High", new AutoHigh().getRoutine());
    climbChooser.addOption("Mid", new AutoMid().getRoutine());
    tab.add(climbChooser);
  }

  @Override
  public void simulationInit() {}

  ShuffleboardTab tab = Shuffleboard.getTab("Auto chooser");

  @Override
  public void disabledInit() {
    mRobotState.gamePeriod = RobotState.GamePeriod.DISABLED;
    resetCommandsAndRoutines();
  }

  @Override
  public void autonomousInit() {
    mHardwareReader.readColorAndGameData(mRobotState);
    startStage(RobotState.GamePeriod.AUTO);
    mCommands.addWantedRoutine(autoChooser.getSelected());
    HardwareAdapter.MiscHardware.getInstance().gyro.zeroYaw();
  }

  private void startStage(RobotState.GamePeriod period) {
    mRobotState.gamePeriod = period;
    resetCommandsAndRoutines();
  }

  @Override
  public void teleopInit() {
    mHardwareReader.readColorAndGameData(mRobotState);
    startStage(RobotState.GamePeriod.TELEOP);
  }

  @Override
  public void testInit() {
    startStage(RobotState.GamePeriod.TESTING);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void simulationPeriodic() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousPeriodic() {
    sLoopDebugger.reset();
    readRobotState();
    mRoutineManager.update(mCommands, mRobotState);
    updateSubsystemsAndApplyOutputs();
    sLoopDebugger.finish();
  }

  @Override
  public void teleopPeriodic() {
    sLoopDebugger.reset();
    readRobotState();
    mOperatorInterface.updateCommands(mCommands, mRobotState);
    mRoutineManager.update(mCommands, mRobotState);
    updateSubsystemsAndApplyOutputs();
    sLoopDebugger.finish();
  }

  @Override
  public void testPeriodic() {
    teleopPeriodic();
  }

  private void resetCommandsAndRoutines() {
    mOperatorInterface.reset(mCommands);
    mRoutineManager.clearRunningRoutines();
    updateSubsystemsAndApplyOutputs();
  }

  private void readRobotState() {
    mHardwareReader.readState(mSubsystems, mRobotState);
  }

  private void resetOdometryIfWanted() {
    Pose2d wantedPose = mCommands.driveWantedOdometryPose;
    Rotation2d wantedPoseRotation = mCommands.driveWantedOdometryPoseRotation;
    if (wantedPose != null && wantedPoseRotation != null) {
      mRobotState.driveOdometry.resetPosition(wantedPose, wantedPoseRotation);
      mRobotState.realPose = wantedPose;
      mCommands.driveWantedOdometryPose = null;
      mCommands.driveWantedOdometryPoseRotation = null;
    }
  }

  private void updateSubsystemsAndApplyOutputs() {
    resetOdometryIfWanted();
    for (SubsystemBase subsystem : mSubsystems) {
      subsystem.update(mCommands, mRobotState);
      sLoopDebugger.addPoint(subsystem.getName());
    }
    mHardwareWriter.writeHardware(mSubsystems, mRobotState);
    sLoopDebugger.addPoint("updateSubsystemsAndApplyOutputs");
  }
}

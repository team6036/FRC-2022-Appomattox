package com.swervedrivespecialties.swervelib;

public final class SdsModuleConfigurations {

  public static final ModuleConfiguration MK3_STANDARD =
      new ModuleConfiguration(
          0.1016,
          (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0),
          true,
          (15.0 / 32.0) * (10.0 / 60.0),
          true);
  public static final ModuleConfiguration MK3_FAST =
      new ModuleConfiguration(
          0.1016,
          (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 60.0),
          true,
          (15.0 / 32.0) * (10.0 / 60.0),
          true);
  public static final ModuleConfiguration MK3_FAST_WITH_TREAD =
      new ModuleConfiguration(
          0.091 + 0.003,
          (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 60.0),
          true,
          (15.0 / 32.0) * (10.0 / 60.0),
          true); // EDIT
  //	public static final ModuleConfiguration MK3_FAST_WITH_TREAD = new ModuleConfiguration(
  //			0.091 - 0.002,
  //			(16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 60.0),
  //			true,
  //			(15.0 / 32.0) * (10.0 / 60.0),
  //			true); //EDIT
  public static final ModuleConfiguration MK4_L1 =
      new ModuleConfiguration(
          0.10033,
          (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
          true,
          (15.0 / 32.0) * (10.0 / 60.0),
          true);
  public static final ModuleConfiguration MK4_L2 =
      new ModuleConfiguration(
          0.10033,
          (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
          true,
          (15.0 / 32.0) * (10.0 / 60.0),
          true);
  public static final ModuleConfiguration MK4_L3 =
      new ModuleConfiguration(
          0.10033,
          (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0),
          true,
          (15.0 / 32.0) * (10.0 / 60.0),
          true);
  public static final ModuleConfiguration MK4_L4 =
      new ModuleConfiguration(
          0.10033,
          (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 45.0),
          true,
          (15.0 / 32.0) * (10.0 / 60.0),
          true);

  private SdsModuleConfigurations() {}
}
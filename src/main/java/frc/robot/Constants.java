// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.Collection;
import java.util.List;

/** Add your docs here. */
public class Constants {

  public static final String kCANBUSNAME = "canivore";

  public class GroundIntakeConstants {

    public static final double kS = 0.15;
    public static final double kG = 0.25;
    public static final double kV = 0.23;
    public static final double kP = 3.2;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public double intakeSpeed = 0.0;
    public double encoderPosition = 0.0;

    public static final double MIN_WHEEL_SPEED = 0;
    public static final double MIN_PIVOT_SPEED = 0;
    public static final double feedingIntakeSpeed = 0.0;
    public static final double kINTAKING_POSITION_SETPOINT = 5.35;
    public static final double kIDLED_POSITION_SETPOINT = 5.35;
    public static final double kZERO_SETPOINT = 0.0;
    public static final double kINTAKE_MAX_ANGLE_UNDER_TRENCH = 5.0;
    public static final double kENCODER_OFFSET = 0.313;
    public static final int kENCODER_PORT = 9;
  }

  public class ShooterConstants {

    public static final double kSHOOTER_SPEED_AT_HUB = 5300.0;
    public static final double kSHOOTER_SPEED_TOLERANCE = 100.;
    public static final double kREVERSING_SPEED = 0.0;
    public static final double kMAX_DISTANCE_FROM_HUB = 2.8;
    public static final double kMIN_DISTANCE_FROM_HUB = 1.51;
    public static final InterpolatingDoubleTreeMap kSHOOTER_SPEEDS =
      new InterpolatingDoubleTreeMap();
    public static final Double[] kSHOOTER_ENTRY_000 = new Double[] {
      1.3,
      3850.,
    };
    public static final Double[] kSHOOTER_ENTRY_001 = new Double[] {
      1.4,
      3870.,
    };
    public static final Double[] kSHOOTER_ENTRY_0002 = new Double[] {
      1.5,
      3900.,
    };
    public static final Double[] kSHOOTER_ENTRY_00 = new Double[] {
      1.6,
      4080.,
    };
    public static final Double[] kSHOOTER_ENTRY_01 = new Double[] {
      1.7,
      4300.,
    };
    public static final Double[] kSHOOTER_ENTRY_02 = new Double[] {
      1.8,
      4500.,
    };
    public static final Double[] kSHOOTER_ENTRY_03 = new Double[] {
      1.9,
      4600.,
    };
    public static final Double[] kSHOOTER_ENTRY_04 = new Double[] {
      2.0,
      4620.,
    };
    public static final Double[] kSHOOTER_ENTRY_05 = new Double[] {
      2.1,
      4775.,
    };
    public static final Double[] kSHOOTER_ENTRY_06 = new Double[] {
      2.2,
      5150.0,
    };
    public static final Double[] kSHOOTER_ENTRY_07 = new Double[] {
      2.3,
      5250.0,
    };
    public static final Double[] kSHOOTER_ENTRY_08 = new Double[] {
      2.4,
      5350.0,
    };
    public static final Double[] kSHOOTER_ENTRY_09 = new Double[] {
      2.5,
      5550.0,
    };
    public static final Double[] kSHOOTER_ENTRY_10 = new Double[] {
      2.6,
      5650.0,
    };
    public static final Double[] kSHOOTER_ENTRY_11 = new Double[] {
      2.7,
      5750.0,
    };
    public static final Double[] kSHOOTER_ENTRY_12 = new Double[] {
      2.8,
      5800.0,
    };

    public static final Double[] kSHOOTER_ENTRY_13 = new Double[] {
      2.9,
      5900.0,
    };

    public static final Double[] kSHOOTER_ENTRY_1 = new Double[] {
      3.0,
      6100.0,
    };

    public static final InterpolatingDoubleTreeMap k_SHOOTER_SPEED_MAP_DCMP = new InterpolatingDoubleTreeMap();
    public static final Double[] kSHOOTER_ENTRY_HUB_DCMP = new Double[] {
      1.75, 4200.
    };
    public static final Double[] k_SHOOTER_ENTRY_FAR_DCMP = new Double[] {
      3.5, 6150.
    };
    public static final double kSHOOTER_REV_ADJUSTMENT = 80.;
    public static final double kP = 0.4;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.105;
    public static final double kS = 0.0;
    public static final double kA = 0.0;

    public static final double kHOOD_P = 2.35;
    public static final double kHOOD_I = 0;
    public static final double kHOOD_D = 0;
    public static final double kHOOD_V = 1.85;
    public static final double kHOOD_S = 0.025;
    public static final double kHOOD_A = 0;

    public static final double kHOOD_UP_ROTATIONS = 0.276;

    public static final InterpolatingDoubleTreeMap kHOOD_SPEED_MAP = new InterpolatingDoubleTreeMap();
  }

  public class HopperConstants {

    public static final double kBELT_SPEED = 0.0;
  }

  public class VisionConstants {

    public static final int kLED_PORT = 3;
    public static final String kAPRILTAG_LL_NAME = "limelight-intake";
    public static final Translation2d kBLUEHUBPOSE = new Translation2d(
      4.634,
      4.029
    );
    public static final Translation2d kREDHUBPOSE = new Translation2d(
      11.919,
      4.029
    );

    public static final double[] kTRENCH_GOING_OUT_SETPOINTS = new double[] {
      -18,
      -1,
      0,
    };
    public static final double[] kTRENCH_GOING_IN_SETPOINTS = new double[] {
      0,
      0,
      180,
    };

    public class TidalLockConstants {

      public static final double kP = 0.08;
      public static final double kI = 0.;
      public static final double kD = 0.0;
      public static final double kVELOCITY_MULTIPLIER = 2.5;
    }
  }

  public class MotorIDConstants {

    public static final int kPIVOT_ID = 35;
    public static final int kINTAKE_ID1 = 36;
    public static final int kINTAKE_ID2 = 37;
    public static final int kHOP_ID = 20;
    public static final int kSHOOTER_ID = 51;
    public static final int kLOADER_ID = 50;
    public static final int kSHOOTER_FOLLOWER_ID = 52;
    public static final int kLOADER_FOLLOWER_ID = 53;
    public static final int kHOOD_ID = 55;
  }
}

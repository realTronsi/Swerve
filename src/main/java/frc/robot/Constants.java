// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /** */
  public static final class CANIDS {
    public static final int FrontLeftSteer = 1;
    public static final int FrontLeftDrive = 2;
    public static final int FrontLeftCANCoder = 3;

    public static final int FrontRightSteer = 4;
    public static final int FrontRightDrive = 5;
    public static final int FrontRightCANCoder = 6;

    public static final int BackLeftSteer = 7;
    public static final int BackLeftDrive = 8;
    public static final int BackLeftCANCoder = 9;

    public static final int BackRightSteer = 10;
    public static final int BackRightDrive = 11;
    public static final int BackRightCANCoder = 12;
  }
  /** */
  public static final class PIDS {
    public static final class FrontLeftSteer {
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kV = 0.0; // kFF
    }
    public static final class FrontLeftDrive {
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kV = 0.0; // kFF
    }
    
    public static final class FrontRightSteer {
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kV = 0.0; // kFF
    }
    public static final class FrontRightDrive {
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kV = 0.0; // kFF
    }
    
    public static final class BackLeftSteer {
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kV = 0.0; // kFF
    }
    public static final class BackLeftDrive {
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kV = 0.0; // kFF
    }

    public static final class BackRightSteer {
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kV = 0.0; // kFF
    }
    public static final class BackRightDrive {
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kV = 0.0; // kFF
    }
  }
  /** */
  public static final class DriveTrainConstants {

    public static final double MAX_SPEED = 0.0; // m/s

    public static final class SwerveModuleConstants {
      public static final double AT_ANGLE_TOLERANCE = 0.0;
      public static final double AT_POSITION_TOLERANCE = 0.0;
      public static final double IS_TURNING_TOLERANCE = 0.0;
    }


    public static final class FrontLeftModule {
      public static final String name = "FLM";
      public static final Translation2d pos = new Translation2d(0.0, 0.0);
    }

    public static final class FrontRightModule {
      public static final String name = "FRM";
      public static final Translation2d pos = new Translation2d(0.0, 0.0);
    }

    public static final class BackLeftModule {
      public static final String name = "BLM";
      public static final Translation2d pos = new Translation2d(0.0, 0.0);
    }

    public static final class BackRightModule {
      public static final String name = "BRM";
      public static final Translation2d pos = new Translation2d(0.0, 0.0);
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.PIDS;
import frc.robot.Constants.DriveTrainConstants.SwerveModuleConstants;
import frc.robot.utility.MathPP;

public class DriveTrain extends SubsystemBase {
  private static final DriveTrain instance = new DriveTrain();
  /**
   * 
   * @return drivetrain instance
   */
  public static DriveTrain getInstance() {
    return instance;
  }

  private final NavX navX = RobotContainer.navX;
  
  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  /** Creates a new DriveTrain. */
  private DriveTrain() {
    // Initialize modules
    // Not worth it to create forEach method or config class

    /** FRONT LEFT */
    frontLeftModule = new SwerveModule(
      DriveTrainConstants.FrontLeftModule.name,
      CANIDS.FrontLeftSteer,
      CANIDS.FrontLeftDrive,
      CANIDS.FrontLeftCANCoder
    );
    frontLeftModule.getSteerController().setPIDF(
      PIDS.FrontLeftSteer.kP,
      PIDS.FrontLeftSteer.kI,
      PIDS.FrontLeftSteer.kD,
      PIDS.FrontLeftSteer.kV
    );
    frontLeftModule.getDriveController().setPIDF(
      PIDS.FrontLeftDrive.kP,
      PIDS.FrontLeftDrive.kI,
      PIDS.FrontLeftDrive.kD,
      PIDS.FrontLeftDrive.kV
    );

    /** FRONT RIGHT */
    frontRightModule = new SwerveModule(
      DriveTrainConstants.FrontRightModule.name,
      CANIDS.FrontRightSteer,
      CANIDS.FrontRightDrive,
      CANIDS.FrontRightCANCoder
    );
    frontRightModule.getSteerController().setPIDF(
      PIDS.FrontRightSteer.kP,
      PIDS.FrontRightSteer.kI,
      PIDS.FrontRightSteer.kD,
      PIDS.FrontRightSteer.kV
    );
    frontRightModule.getDriveController().setPIDF(
      PIDS.FrontRightDrive.kP,
      PIDS.FrontRightDrive.kI,
      PIDS.FrontRightDrive.kD,
      PIDS.FrontRightDrive.kV
    );
    
    /** BACK LEFT */
    backLeftModule = new SwerveModule(
      DriveTrainConstants.BackLeftModule.name,
      CANIDS.BackLeftSteer,
      CANIDS.BackLeftDrive,
      CANIDS.BackLeftCANCoder
    );
    backLeftModule.getSteerController().setPIDF(
      PIDS.BackLeftSteer.kP,
      PIDS.BackLeftSteer.kI,
      PIDS.BackLeftSteer.kD,
      PIDS.BackLeftSteer.kV
    );
    backLeftModule.getDriveController().setPIDF(
      PIDS.BackLeftDrive.kP,
      PIDS.BackLeftDrive.kI,
      PIDS.BackLeftDrive.kD,
      PIDS.BackLeftDrive.kV
    );

    /** BACK RIGHT */
    backRightModule = new SwerveModule(
      DriveTrainConstants.BackRightModule.name,
      CANIDS.BackRightSteer,
      CANIDS.BackRightDrive,
      CANIDS.BackRightCANCoder
    );
    backRightModule.getSteerController().setPIDF(
      PIDS.BackRightSteer.kP,
      PIDS.BackRightSteer.kI,
      PIDS.BackRightSteer.kD,
      PIDS.BackRightSteer.kV
    );
    backRightModule.getDriveController().setPIDF(
      PIDS.BackRightDrive.kP,
      PIDS.BackRightDrive.kI,
      PIDS.BackRightDrive.kD,
      PIDS.BackRightDrive.kV
    );

    /** */
    kinematics = new SwerveDriveKinematics(
      DriveTrainConstants.FrontLeftModule.pos,
      DriveTrainConstants.FrontRightModule.pos,
      DriveTrainConstants.BackLeftModule.pos,
      DriveTrainConstants.BackRightModule.pos
    );

    odometry = new SwerveDriveOdometry(kinematics, navX.getYawAsRotation2d());
  }

  /** */
  public void resetOdometry() {
    odometry.resetPosition(odometry.getPoseMeters(), navX.getYawAsRotation2d());
  }

  /** */
  public void resetOdometry(Pose2d pose, Rotation2d rot) {
    odometry.resetPosition(pose, rot);
  }

  /**
   * 
   * @param vx m/s
   * @param vy m/s
   * @param omega deg/s
   */
  public void openLoopDrive(
    double vx,
    double vy,
    double omega
  ) {
    // Open loop is unreliable when turning
    if (omega != 0 || isAnyModuleTurning()) {
      closedLoopDrive(vx, vy, omega);
      return;
    }

    double speed = Math.hypot(vx, vy);
    speed = Math.min(speed, DriveTrainConstants.MAX_SPEED);

    frontLeftModule.getDriveController().set(speed);
    frontRightModule.getDriveController().set(speed);
    backLeftModule.getDriveController().set(speed);
    backRightModule.getDriveController().set(speed);
  }
  
  /** */
  public void closedLoopDrive(
    double vx,
    double vy,
    double omega
  ) {
    ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, MathPP.degToRad(omega));
    setModuleStates(kinematics.toSwerveModuleStates(speeds));
  }

  /**
   * 
   * @param desiredStates
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Scale wheel speeds below maximum possible speed
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveTrainConstants.MAX_SPEED);

    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }

  /** */
  public boolean isAnyModuleTurning() {
    return (
      frontLeftModule.getSteerController().getVelocity() > SwerveModuleConstants.IS_TURNING_TOLERANCE ||
      frontRightModule.getSteerController().getVelocity() > SwerveModuleConstants.IS_TURNING_TOLERANCE ||
      backLeftModule.getSteerController().getVelocity() > SwerveModuleConstants.IS_TURNING_TOLERANCE ||
      backRightModule.getSteerController().getVelocity() > SwerveModuleConstants.IS_TURNING_TOLERANCE
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void periodic10ms() {
    frontLeftModule.periodic10ms();
    frontRightModule.periodic10ms();
    backLeftModule.periodic10ms();
    backRightModule.periodic10ms();

    odometry.update(
      navX.getYawAsRotation2d(), 
      frontLeftModule.getState(),
      frontRightModule.getState(),
      backLeftModule.getState(),
      backRightModule.getState()
    );
  }
}

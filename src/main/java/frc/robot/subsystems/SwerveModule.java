// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.REVLibError;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.CANMotorController;
import frc.robot.utility.MathPP;

public class SwerveModule extends SubsystemBase {
  private final String moduleName;
  private final CANMotorController steerController;
  private final CANMotorController driveController;
  private final CANCoder canCoder;

  /**
   * 
   * @param name
   * @param steerID
   * @param driveID
   * @param coderID
   */
  public SwerveModule(
    String name,
    int steerID,
    int driveID,
    int coderID
  ) {
    moduleName = name;
    steerController = new CANMotorController(steerID);
    driveController = new CANMotorController(driveID);
    canCoder = new CANCoder(coderID);

    canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    canCoder.setPositionToAbsolute();
    // [0, 360)
    canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    canCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10);
  }

  /**
   * 
   * Less aggressive swerve optimization using 120 degrees instead of 90
   * 
   * @param desiredState module state to be optimized
   * @param currentAngle current module angle
   * @return optimized module state
   */
  public static final SwerveModuleState optimizeState(
    SwerveModuleState desiredState,
    double currentAngle
  ) {
    double desiredAngle = desiredState.angle.getDegrees();
    double desiredVelocity = desiredState.speedMetersPerSecond;

    // Angle difference is less than 120, no optimization to be done
    if (MathPP.absAngleDiff(desiredAngle, currentAngle) <= 120) return desiredState;

    // Optimized state
    return new SwerveModuleState(
      // Reverse drive direction
      -desiredVelocity,
      // Use directly opposite angle
      Rotation2d.fromDegrees(MathPP.toUnitAngle(desiredAngle) - 180)
    );
  }

  /** */
  public CANMotorController getSteerController() {
    return steerController;
  }

  /** */
  public CANMotorController getDriveController() {
    return driveController;
  }

  /** */
  public CANCoder getEncoder() {
    return canCoder;
  }

  /** */
  public double getAngle() {
    return canCoder.getAbsolutePosition();
  }

  /** */
  public Rotation2d getAngleAsRotation2d() {
    return Rotation2d.fromDegrees(this.getAngle());
  }

  /** */
  public double getVelocity() {
    return driveController.getVelocity();
  }

  /**
   * 
   * @param value deg
   * @return
   */
  public REVLibError setDesiredAngle(double value) {
    return steerController.setDesiredPosition(value);
  }

  /**
   * 
   * @param value deg/s
   * @return
   */
  public REVLibError setDesiredVelocity(double value) {
    return driveController.setDesiredVelocity(value);
  }

  /**
   * 
   * @param desiredState
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    desiredState = SwerveModule.optimizeState(desiredState, this.getAngle());

    this.setDesiredAngle(desiredState.angle.getDegrees());
    this.setDesiredVelocity(desiredState.speedMetersPerSecond);
  }

  /** */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      this.getVelocity(),
      this.getAngleAsRotation2d()
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** */
  public void periodic10ms() {
    steerController.getEncoder().setPosition(this.getAngle());
  }
}

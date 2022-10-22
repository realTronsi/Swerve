// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class NavX extends SubsystemBase {
  private static final NavX instance = new NavX();
  public static final NavX getInstance() {
    return instance;
  }

  private final AHRS ahrs;
  /** Creates a new NavX. */
  private NavX() {
    ahrs = new AHRS(SPI.Port.kMXP);
    ahrs.calibrate();
  }

  /**
   * Reset yaw value to 0
   */
  public void zeroYaw() {
    ahrs.zeroYaw();
    RobotContainer.driveTrain.resetOdometry();
  }
  /**
   * 
   * Negate because gyros use cw = (+) while we want ccw (+)
   * 
   * @return yaw of gyro
   */
  public double getYaw() {
    return -ahrs.getYaw();
  }

  /** */
  public Rotation2d getYawAsRotation2d() {
    return Rotation2d.fromDegrees(this.getYaw());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

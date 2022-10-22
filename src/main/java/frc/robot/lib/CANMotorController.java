package frc.robot.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

/**
 * Lightweight CANSparkMax wrapper
 */
public class CANMotorController {

  private final CANSparkMax sparkMax;
  private final SparkMaxPIDController sparkMaxPID;
  private final RelativeEncoder sparkMaxEncoder;

  // Caching PID values
  private PIDState lastState;
  private REVLibError lastStatus;

  /**
   * Helper class to store and compare PID setpoint states
   */
  private static final class PIDState {
    public final double value;
    public final CANSparkMax.ControlType controlType;

    public static PIDState fromVelocity(double value) {
      return new PIDState(value, CANSparkMax.ControlType.kVelocity);
    }

    public static PIDState fromPosition(double value) {
      return new PIDState(value, CANSparkMax.ControlType.kPosition);
    }

    public PIDState(
      double value,
      CANSparkMax.ControlType controlType
    ) {
      this.value = value;
      this.controlType = controlType;
    }

    public boolean equals(PIDState state) {
      if (state == null) return false;
      return
        this.value == state.value &&
        this.controlType.equals(state.controlType);
    }
  }

  /**
   * 
   * Constructor
   * 
   * @param CANID device ID
   */
  public CANMotorController(int CANID) {
    sparkMax = new CANSparkMax(CANID, CANSparkMax.MotorType.kBrushless);
    sparkMaxPID = sparkMax.getPIDController();
    sparkMaxEncoder = sparkMax.getEncoder();
    
    /* Defaults */
    sparkMax.restoreFactoryDefaults();

    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);

    sparkMax.setInverted(false);
    sparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
    sparkMax.enableVoltageCompensation(10.0);
    sparkMax.setSmartCurrentLimit(20);

    // RPM -> deg/s
    sparkMaxEncoder.setVelocityConversionFactor(6);
    // Rot -> deg
    sparkMaxEncoder.setPositionConversionFactor(360);
  }

  /**
   * 
   * @return CANSparkMax object
   */
  public CANSparkMax getController() {
    return sparkMax;
  }

  /** */
  public SparkMaxPIDController getPIDController() {
    return sparkMaxPID;
  }

  /** */
  public RelativeEncoder getEncoder() {
    return sparkMaxEncoder;
  }

  /** */
  public void resetCache() {
    this.lastState = null;
    this.lastStatus = null;
  }

  public void stop() {
    this.resetCache();
    this.set(0);
  }

  /** 
   * Lazy method for setting all PID constants
   */
  public void setPIDF(
    double kP,
    double kI,
    double kD,
    double kFF
  ) {
    sparkMaxPID.setP(kP);
    sparkMaxPID.setI(kI);
    sparkMaxPID.setD(kD);
    sparkMaxPID.setFF(kFF);

    sparkMax.burnFlash();
  }

  /**
   * 
   * @param speed m/s
   */
  public void set(double speed) {
    sparkMax.set(speed);
  }

  /**
   * 
   * @param value deg/s
   * @return
   */
  public REVLibError setDesiredVelocity(double value) {
    PIDState desiredState = PIDState.fromVelocity(value);
    // Identical to previous PID setpoint, no need to resend
    if (desiredState.equals(this.lastState)) return lastStatus;

    lastState = desiredState;
    return lastStatus = sparkMaxPID.setReference(value, CANSparkMax.ControlType.kVelocity);
  }

  /** */
  public double getVelocity() {
    return sparkMaxEncoder.getVelocity();
  }

  /**
   * 
   * @param value deg
   * @return
   */
  public REVLibError setDesiredPosition(double value) {
    PIDState desiredState = PIDState.fromPosition(value);
    // Identical to previous PID setpoint, no need to resend
    if (desiredState.equals(this.lastState)) return lastStatus;

    lastState = desiredState;
    return lastStatus = sparkMaxPID.setReference(value, CANSparkMax.ControlType.kPosition);
  }

  /** */
  public double getPosition() {
    return sparkMaxEncoder.getPosition();
  }
}

package frc.robot.subsystems.GroundIntake;

import static frc.robot.Constants.GroundIntakeConstants.*;
import static frc.robot.Constants.kCANBUSNAME;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GroundIntakeIOTalonFX implements GroundIntakeIO {

  private final TalonFX pivotMotor;
  private final TalonFX intakeMotor;
  private final TalonFX intakeMotorFollower;
  private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(
    kENCODER_PORT
  );
  MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

  public GroundIntakeIOTalonFX(int pivotID, int intakeID, int followerID) {
    this.pivotMotor = new TalonFX(pivotID, kCANBUSNAME);
    this.intakeMotor = new TalonFX(intakeID, kCANBUSNAME);
    this.intakeMotorFollower = new TalonFX(followerID, kCANBUSNAME);
    pivotMotor.setPosition(0);
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;
    slot0Configs.kV = kV;
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.StatorCurrentLimit = 90;
    currentLimitsConfigs.SupplyCurrentLimit = 50;
    CurrentLimitsConfigs currentLimitsConfigsPivot = new CurrentLimitsConfigs();
    currentLimitsConfigsPivot.StatorCurrentLimit = 80;
    currentLimitsConfigsPivot.SupplyCurrentLimit = 40;
    pivotMotor.getConfigurator().apply(currentLimitsConfigsPivot);
    pivotMotor.getConfigurator().apply(slot0Configs);
    SoftwareLimitSwitchConfigs limits = new SoftwareLimitSwitchConfigs();

    limits.ForwardSoftLimitEnable = true;
    limits.ForwardSoftLimitThreshold = 5.93;

    limits.ReverseSoftLimitEnable = true;
    limits.ReverseSoftLimitThreshold = -0.04;

    MotionMagicConfigs mm = new MotionMagicConfigs();
    mm.MotionMagicCruiseVelocity = 20; // rotations/sec
    mm.MotionMagicAcceleration = 40; // rotations/sec^2
    mm.MotionMagicJerk = 0; // optional

    MotorOutputConfigs output = new MotorOutputConfigs();
    output.Inverted = InvertedValue.Clockwise_Positive;
    pivotMotor.getConfigurator().apply(mm);
    intakeMotor.getConfigurator().apply(currentLimitsConfigs);
    intakeMotorFollower.getConfigurator().apply(currentLimitsConfigs);
    pivotMotor.getConfigurator().apply(limits);

    // pivotMotor.getConfigurator().apply(output);
  }

  @Override
  public void setIntakeSpeed(double speed) {
    this.intakeMotor.set(speed);
    this.intakeMotorFollower.set(speed);
  }

  @Override
  public void setPivotSpeed(double speed) {
    this.pivotMotor.set(speed);
  }

  @Override
  public void updateInputs(GroundIntakeIOInputs inputs) {
    //SPARK MAX VERSION: inputs.intakeSpeed = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
    inputs.intakeSpeed = intakeMotor.get();
    inputs.pivotSpeed = pivotMotor.get();
    //SPARK MAX VERSION: inputs.encoderPosition = pivotMotor.getOutputCurrent();
    inputs.encoderPosition = pivotMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void refreshData() {
    // Not required for Spark MAX, but useful for manual telemetry push or debug logging
    //SPARK MAX VERSION: SmartDashboard.putNumber("GroundIntake/WheelSpeed", intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage());
    //SPARK MAX VERSION: SmartDashboard.putNumber("GroundIntake/EncoderPosition", pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage());
    SmartDashboard.putNumber("Average Supply Current", (intakeMotor.getSupplyCurrent().getValueAsDouble() + intakeMotorFollower.getSupplyCurrent().getValueAsDouble()) / 2);

    SmartDashboard.putNumber("Top Motor Supply Current", intakeMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Bottom Motor Supply Current", intakeMotorFollower.getSupplyCurrent().getValueAsDouble());
  }

  /* @Override
    public double getEncoderVal() {
      return this.pivotMotor.getEncoder().getPosition();
    } */

  @Override
  public double getIntakePosition() {
    // getPosition() returns a StatusSignal; getValueAsDouble() converts it to a numeric value.
    return pivotMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void runIntakePivotToSetpoint(double setpoint) {
    pivotMotor.setControl(motionMagicVoltage.withPosition(setpoint));
  }

  public double getAbsEncoderVal() {
    return absEncoder.get() - kENCODER_OFFSET;
  }

  @Override
  public double getIntakeCurrentAverage() {
    return (intakeMotor.getStatorCurrent().getValueAsDouble() + intakeMotorFollower.getStatorCurrent().getValueAsDouble()) / 2;
  }
}

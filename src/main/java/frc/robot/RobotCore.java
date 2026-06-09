// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.GroundIntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GroundIntake.GroundIntakeSubsystem;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.util.Limelight;
import frc.robot.util.ShooterAlgorithm;

/** Add your docs here. */
public class RobotCore extends SubsystemBase {

  ShooterSubsystem shooter;
  GroundIntakeSubsystem groundIntake;
  HopperSubsystem hopper;
  CommandSwerveDrivetrain drivetrain;
  LEDs leds;
  Limelight tagLimelight;
  ShooterAlgorithm shooterAlgorithm = new ShooterAlgorithm();
  private WantedSuperState wantedSuperState = WantedSuperState.IDLE;
  private CurrentSuperState currentSuperState = CurrentSuperState.IDLING;
  private boolean readyToShoot = false;
  private boolean intakeOverride = false;
  private boolean shootingAtHub = true;
  private double shooterCalculatedSpeed = 0.0;
  private boolean hasResetPoseToShoot = false;
  private boolean hasCalculatedShooterSpeed = false;
  private boolean toggleRevving = true;
  private boolean wiggle = false;
  private boolean shouldSetStaticPoseAuto = false;
  double cyclesStill = 0;
  Pose2d lastCyclePose;
  double hoodAngle = 0;
  double timeOfStartedShooting;
  private boolean hasCalculatedTimeOfShooting = false;
  double manualAdjust = 0;

  public RobotCore(
    ShooterSubsystem shooter,
    GroundIntakeSubsystem groundIntake,
    HopperSubsystem hopper,
    CommandSwerveDrivetrain drivetrain,
    LEDs leds,
    Limelight tagLimelight
  ) {
    this.shooter = shooter;
    this.groundIntake = groundIntake;
    this.hopper = hopper;
    this.drivetrain = drivetrain;
    this.leds = leds;
    this.tagLimelight = tagLimelight;
    SmartDashboard.putNumber("shooter speed", kSHOOTER_SPEED_AT_HUB);
  }

  public enum WantedSuperState {
    IDLE,
    SHOOT,
    SHOOT_FROM_DISTANCE,
    INTAKE,
    REVERSE_INTAKE,
    HOME,
    REV_AUTO,
    SHOOT_WHILE_MOVING,
  }

  public enum CurrentSuperState {
    IDLING,
    SHOOTING,
    SHOOTING_FROM_DISTANCE,
    INTAKING,
    REVERSING_INTAKE,
    HOMED,
    REVVING_AUTO,
    SHOOTING_WHILE_MOVING,
  }

  @Override
  public void periodic() {
    handleStateTransitions();
    applyStateBehavior();
    if (shooter.isUpToSpeed()) {
      readyToShoot = true;
    } else {
      readyToShoot = false;
    }
    SmartDashboard.putBoolean("Superstructure/Ready to Shoot", readyToShoot);
    SmartDashboard.putBoolean("Superstructure/Intake Override", intakeOverride);
    SmartDashboard.putNumber(
      "Superstructure/Shooter Speed Setpoint",
      shooterCalculatedSpeed
    );
    SmartDashboard.putBoolean(
      "Superstructure/Is Shooting At Hub",
      shootingAtHub
    );
    SmartDashboard.putBoolean(
      "Has Calculated time",
      hasCalculatedTimeOfShooting
    );
    SmartDashboard.putString(
      "Superstructure/Current State",
      currentSuperState.toString()
    );
    SmartDashboard.putBoolean("wiggle", wiggle);
  }

  public void handleStateTransitions() {
    switch (wantedSuperState) {
      case IDLE:
        currentSuperState = CurrentSuperState.IDLING;
        break;
      case SHOOT:
        currentSuperState = CurrentSuperState.SHOOTING;
        break;
      case SHOOT_FROM_DISTANCE:
        currentSuperState = CurrentSuperState.SHOOTING_FROM_DISTANCE;
        break;
      case INTAKE:
        currentSuperState = CurrentSuperState.INTAKING;
        break;
      case REVERSE_INTAKE:
        currentSuperState = CurrentSuperState.REVERSING_INTAKE;
        break;
      case HOME:
        currentSuperState = CurrentSuperState.HOMED;

        break;
      case REV_AUTO:
        currentSuperState = CurrentSuperState.REVVING_AUTO;
        break;
      case SHOOT_WHILE_MOVING:
        if (drivetrain.getVelocityVector() < 0.25) currentSuperState =
          CurrentSuperState.SHOOTING_FROM_DISTANCE;
        else currentSuperState = CurrentSuperState.SHOOTING_WHILE_MOVING;
        break;
      default:
        currentSuperState = CurrentSuperState.HOMED;
    }
  }

  public void applyStateBehavior() {
    switch (currentSuperState) {
      case IDLING:
        hopper.stop();
        shooter.stop();
        break;
      case SHOOTING:
        shooter.shoot(4500, 0.2);
        if (shooter.isUpToSpeed()) {
          if (!intakeOverride) {
            shooter.shootAtHub();
          } else {
            shooter.shootAtHubRegular();
          }
          hopper.setWantedState(HopperSubsystem.WantedState.FEED);
        }
        break;
      case SHOOTING_FROM_DISTANCE:
        if (!hasCalculatedShooterSpeed || DriverStation.isAutonomous() || DriverStation.isTeleop()) {
          var cameraPose = tagLimelight.getLimelightPoseEstimateData().pose;
          if (cameraPose != null && cameraPose.getTranslation().getNorm() > 0.1) {
            drivetrain.resetGlobalPose(
              cameraPose
            );
          }
          shooterCalculatedSpeed = shooterAlgorithm.calculateShooterSpeedDCMP(
            drivetrain.getDistanceFromHub()
          );
          hoodAngle = shooterAlgorithm.calculateHoodAngle(
            drivetrain.getDistanceFromHub()
          );
          hasCalculatedShooterSpeed = true;
        }
        shooter.shoot(
          shooterCalculatedSpeed + kSHOOTER_REV_ADJUSTMENT,
          hoodAngle
        );
        if (shooter.isUpToSpeed()) {
          hopper.setWantedState(HopperSubsystem.WantedState.FEED);
          shooter.feedAndShoot(
            shooterCalculatedSpeed + manualAdjust,
            hoodAngle
          );
        }
        break;
      case SHOOTING_WHILE_MOVING:
        if (!hasResetPoseToShoot) {
          if (tagLimelight.isSeeingValidTarget()) {
            drivetrain.resetGlobalPose(
              tagLimelight.getLimelightPoseEstimateData().pose
            );
          }
          hasResetPoseToShoot = true;
        }
        double setpoint =
          shooterAlgorithm.calculateShooterSpeedRegular(
            drivetrain.getDistanceFromHub()
          ) +
          drivetrain.getRateOfChangeOfDistanceFromHubMetersPerSecond() * 200;
        double hoodAngleMoving = shooterAlgorithm.calculateHoodAngle(setpoint);
        shooter.feedAndShoot(setpoint, hoodAngleMoving);
        hopper.feed(0.5);
        SmartDashboard.putNumber("Setpoint", setpoint);
        break;
      case REVVING_AUTO:
        shooter.setWantedState(ShooterSubsystem.WantedState.REV_TO_SPEED);
        hopper.setWantedState(HopperSubsystem.WantedState.IDLE);
        break;
      case INTAKING:
        shooter.setWantedState(ShooterSubsystem.WantedState.IDLE);
        hopper.setWantedState(HopperSubsystem.WantedState.IDLE);
        break;
      case REVERSING_INTAKE:
        hopper.setWantedState(HopperSubsystem.WantedState.REVERSE);
        shooter.stop();
        break;
      case HOMED:
        hasResetPoseToShoot = false;
        hasCalculatedShooterSpeed = false;
        hopper.setWantedState(HopperSubsystem.WantedState.IDLE);
        if (
          (drivetrain.isInAllianceZone(DriverStation.getAlliance().get()) &&
            toggleRevving) ||
          DriverStation.isAutonomous()
        ) {
          shooter.setWantedState(ShooterSubsystem.WantedState.WARM_UP);
        } else {
          shooter.stop();
        }
        break;
    }
    if (
      (intakeOverride || wiggle) &&
      currentSuperState != CurrentSuperState.INTAKING
    ) {
      if (!wiggle) {
        groundIntake.setWantedState(
          GroundIntakeSubsystem.WantedState.HOLD_AT_ZERO
        );
      } else {
        if (
          groundIntake.getWantedState() !=
          GroundIntakeSubsystem.WantedState.HOLD_AT_ZERO
        ) {
          groundIntake.wiggle();
        }
      }
    } else {
      switch (currentSuperState) {
        case INTAKING:
          // if (groundIntake.isFull()) {
          //   groundIntake.setWantedState(GroundIntakeSubsystem.WantedState.IDLE);
          // } else {
          //   groundIntake.setWantedState(
          //     GroundIntakeSubsystem.WantedState.INTAKE
          //   );
          // }
          groundIntake.setWantedState(GroundIntakeSubsystem.WantedState.INTAKE);
          break;
        case REVERSING_INTAKE:
          groundIntake.setWantedState(
            GroundIntakeSubsystem.WantedState.REVERSE
          );
          break;
        case HOMED:
          groundIntake.setWantedState(
            GroundIntakeSubsystem.WantedState.HOLD_AT_DEFAULT
          );
          break;
        case SHOOTING:
          groundIntake.setWantedState(
            GroundIntakeSubsystem.WantedState.COMPRESS
          );
          break;
        case REVVING_AUTO:
          groundIntake.setWantedState(
            GroundIntakeSubsystem.WantedState.HOLD_AT_DEFAULT
          );
          break;
        case SHOOTING_FROM_DISTANCE:
          groundIntake.setWantedState(
            GroundIntakeSubsystem.WantedState.COMPRESS
          );
          break;
        default:
          groundIntake.setWantedState(GroundIntakeSubsystem.WantedState.IDLE);
      }
    }
    leds.setWantedState(computeLedState());
  }

  public CommandSwerveDrivetrain fetchDrivetrain() {
    return drivetrain;
  }

  public Command setWantedSuperStateCommand(WantedSuperState state) {
    return new InstantCommand(() -> this.wantedSuperState = state);
  }

  public void setWantedSuperState(WantedSuperState state) {
    this.wantedSuperState = state;
  }

  public boolean canDriveUnderTrenchSafely() {
    return groundIntake.getIntakePivotAngle() > kINTAKE_MAX_ANGLE_UNDER_TRENCH;
  }

  public Command setIntakeOverrideCommand(boolean override) {
    return new InstantCommand(() -> this.intakeOverride = override);
  }

  public Command toggleAutoRev() {
    return new InstantCommand(() -> this.toggleRevving = (!toggleRevving));
  }

  /**
   * Instant command to set the wanted superstate to SHOOT or SHOOT_FROM_DISTANCE based on whether the robot is currently at the hub or not.
   * @param atHub whether the robot is currently at the hub
   * @return an InstantCommand to set the wanted superstate
   */
  public Command shootFuel(boolean atHub) {
    hasResetPoseToShoot = false;
    if (atHub) return new InstantCommand(() ->
      wantedSuperState = WantedSuperState.SHOOT
    );
    else return new InstantCommand(() ->
      wantedSuperState = WantedSuperState.SHOOT_FROM_DISTANCE
    );
  }

  public CurrentSuperState getCurrentSuperState() {
    return currentSuperState;
  }

  /**
   * Computes the wanted state of the LEDs based on the robot's current state
   * @return the wanted state of the LEDs
   */
  private LEDs.WantedState computeLedState() {
    if (!DriverStation.isEnabled()) return LEDs.WantedState.DISABLED;
    if (
      currentSuperState == CurrentSuperState.SHOOTING ||
      currentSuperState == CurrentSuperState.SHOOTING_FROM_DISTANCE
    ) {
      return LEDs.WantedState.SHOOT_READY;
    }
    if (
      currentSuperState == CurrentSuperState.INTAKING
    ) return !groundIntake.isFull()
      ? LEDs.WantedState.INTAKE
      : LEDs.WantedState.INTAKE_FULL;
    if (
      drivetrain.isInAllianceZone(DriverStation.getAlliance().get()) &&
      drivetrain.getDistanceFromHub() < 3.6 &&
      drivetrain.getDistanceFromHub() > 2.6
    ) return LEDs.WantedState.GREEN;
    if (
      drivetrain.isInAllianceZone(DriverStation.getAlliance().get()) &&
      drivetrain.getDistanceFromHub() < 2.6 &&
      drivetrain.getDistanceFromHub() > 1.75
    ) return LEDs.WantedState.IDEAL;
    return LEDs.WantedState.IDLE;
  }

  /**
   * Toggle the wanted superstate between INTAKE and HOME.
   *
   * @return an InstantCommand that toggles the wanted superstate
   */
  public Command toggleIntake() {
    return new InstantCommand(() -> {
      if (wantedSuperState == WantedSuperState.INTAKE) {
        wantedSuperState = WantedSuperState.HOME;
      } else {
        wantedSuperState = WantedSuperState.INTAKE;
      }
    });
  }

  public boolean cameraSeesTag() {
    return tagLimelight.getLimelightPoseEstimateData().tagCount > 0;
  }

  public Pose2d getLimelightPose() {
    return tagLimelight.getLimelightPoseEstimateData().pose;
  }

  public Command pass() {
    return new InstantCommand(() -> wantedSuperState = WantedSuperState.SHOOT);
  }
  public double getLimelightPoseTimestamp() {
    return tagLimelight.getLimelightPoseEstimateData().timestampSeconds;
  }

  public Command shootWhileMoving() {
    return new InstantCommand(() ->
      this.wantedSuperState = WantedSuperState.SHOOT_WHILE_MOVING
    );
  }

  public Command toggleWiggle() {
    return new InstantCommand(() -> this.wiggle = !this.wiggle);
  }

  public Command editManualAdjust(double adjustment) {
    return new InstantCommand(() -> this.manualAdjust += adjustment);
  }

  public Command resetManualAdjust() {
    return new InstantCommand(() -> this.manualAdjust = 0);
  }
}

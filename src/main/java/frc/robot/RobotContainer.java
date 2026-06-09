// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.GroundIntakeConstants.kS;
import static frc.robot.Constants.MotorIDConstants.*;
import static frc.robot.Constants.VisionConstants.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotCore.CurrentSuperState;
import frc.robot.RobotCore.WantedSuperState;
import frc.robot.commands.BumpLock;
import frc.robot.commands.TidalLockCommand;
import frc.robot.commands.Tweak;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GroundIntake.GroundIntakeIOSim;
import frc.robot.subsystems.GroundIntake.GroundIntakeIOTalonFX;
import frc.robot.subsystems.GroundIntake.GroundIntakeSubsystem;
import frc.robot.subsystems.Hopper.HopperIOSim;
import frc.robot.subsystems.Hopper.HopperIOTalonFX;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.WantedState;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.util.AlliancePhaseDisplay;
import frc.robot.util.Autos;
import frc.robot.util.Limelight;
import frc.robot.util.SubsurfaceDash;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

  private double MaxSpeed =
    1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(
    RadiansPerSecond
  ); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
    new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.06)
      .withRotationalDeadband(MaxAngularRate * 0.06) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.RobotCentric driveRR =
    new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveDriveBrake();
  private final SlewRateLimiter rateLimiter1 = new SlewRateLimiter(13);
  private final SlewRateLimiter rateLimiter2 = new SlewRateLimiter(13);
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  double dTheta = 0;
  double lastThetaEdit = 0;
  public final CommandSwerveDrivetrain drivetrain =
    TunerConstants.createDrivetrain();
  private final ShooterSubsystem shooter = new ShooterSubsystem(
    new ShooterIOTalonFX(
      kSHOOTER_ID,
      kLOADER_ID,
      kSHOOTER_FOLLOWER_ID,
      kLOADER_FOLLOWER_ID,
      kHOOD_ID
    )
  );

  private final GroundIntakeSubsystem groundIntake = new GroundIntakeSubsystem(
    new GroundIntakeIOTalonFX(kPIVOT_ID, kINTAKE_ID1, kINTAKE_ID2)
  );
  private final HopperSubsystem hopper = new HopperSubsystem(
    new HopperIOTalonFX(kHOP_ID)
  );
  SubsurfaceDash dash = new SubsurfaceDash(drivetrain);

  private final LEDs leds = new LEDs(kLED_PORT);
  private final Limelight tagLimelight = new Limelight("limelight-tag");

  private final RobotCore robotSuper = new RobotCore(
    shooter,
    groundIntake,
    hopper,
    drivetrain,
    leds,
    tagLimelight
  );

  private final Autos autos = new Autos(robotSuper, drive);
  private final SendableChooser<Command> autoChooser;
  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("Global Pose", Pose2d.struct)
    .publish();
    StructPublisher<Pose2d> publisherLL = NetworkTableInstance.getDefault()
    .getStructTopic("Limelight Pose", Pose2d.struct)
    .publish();
  private final AlliancePhaseDisplay alliancePhaseDisplay =
    new AlliancePhaseDisplay();

  public RobotContainer() {
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
    drivetrain.configurePigeonMountPose();
    resetGyro();
    drivetrain.initializePoseEstimator();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(
        () ->
          drive
            .withVelocityX(
              rateLimiter1.calculate(-driver.getLeftY() * MaxSpeed)
            ) // Drive forward with negative Y (forward)
            .withVelocityY(
              rateLimiter2.calculate(-driver.getLeftX() * MaxSpeed)
            ) // Drive left with negative X (left) // Drive left with negative X (left)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
      )
    );

    operator.leftTrigger().onTrue(robotSuper.toggleIntake());
    driver
      .rightTrigger()
      .whileTrue(
        new ConditionalCommand(
        new ParallelCommandGroup(
          new TidalLockCommand(
            drivetrain,
            () -> 0,
            () -> 0,
            drive,
            () -> dTheta
          ),
          new SequentialCommandGroup(
            new ParallelRaceGroup(
              new WaitCommand(0.75),
              new WaitUntilCommand(() -> tagLimelight.isSeeingValidTarget())
            ),
            robotSuper.shootFuel(false)
          )
        ),
        robotSuper.pass(),
        () -> drivetrain.isInAllianceZone(DriverStation.getAlliance().get())
        )
      )
      .whileFalse(robotSuper.setWantedSuperStateCommand(WantedSuperState.HOME));
      // driver.leftTrigger().whileTrue(
      //   new TidalLockCommand(drivetrain, () -> driver.getLeftY(), () -> driver.getLeftX(), drive, () -> 0)
      // );
    // driver
    //   .rightTrigger()
    //   .whileTrue(
    //     new ParallelCommandGroup(
    //       new TidalLockCommand(
    //         drivetrain,
    //         () -> driver.getLeftY(),
    //         () -> driver.getLeftX(),
    //         drive
    //       ),
    //       robotSuper.shootWhileMoving()
    //     )
    //   )
    //   .whileFalse(robotSuper.setWantedSuperStateCommand(WantedSuperState.HOME));
    // driver
    //   .y()
    //   .whileTrue(robotSuper.shootFuel(true))
    //   .whileFalse(robotSuper.setWantedSuperStateCommand(WantedSuperState.HOME));
    // driver
    //   .rightBumper()
    //   .whileTrue(
    //     drivetrain.applyRequest(() ->
    //       drive
    //         .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
    //         .withVelocityX(-driver.getLeftY() * MaxSpeed * 0.35)
    //         .withVelocityY(-driver.getLeftX() * MaxSpeed * 0.2)
    //         .withRotationalRate(
    //           dash.calculateRotationalVelocity() * MaxAngularRate
    //         )
    //     )
    //   );
    driver.start().onTrue(new InstantCommand(() -> MaxSpeed *= -1));
    //////OPERATOR CONTROLS ----------------------------------------------------
    operator.b().whileTrue(new Tweak(drivetrain));
    operator
      .leftBumper()
      .whileTrue(robotSuper.setIntakeOverrideCommand(true))
      .whileFalse(robotSuper.setIntakeOverrideCommand(false));
    operator.x().whileTrue(drivetrain.applyRequest(() -> brake));
    operator.start().onTrue(robotSuper.toggleAutoRev());
    operator
      .y()
      .onTrue(new InstantCommand(() -> dTheta = -1))
      .onFalse(new InstantCommand(() -> dTheta = 0));
    operator
      .a()
      .onTrue(new InstantCommand(() -> dTheta = 1))
      .onFalse(new InstantCommand(() -> dTheta = 0));
    operator.povUp().onTrue(
      robotSuper.editManualAdjust(50)
    );
    operator.povDown().onTrue(
      robotSuper.editManualAdjust(-50)
    );
    operator.rightBumper().onTrue(
      robotSuper.resetManualAdjust()
    );
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    // Simple drive forward auton
    return autoChooser.getSelected();
  }

  /**
   * Periodic function that updates various dashboard values and
   * performs actions based on the current state of the robot.
   *
   * @note This function is called periodically by the CommandScheduler.
   */

  public void dashboardUpdates() {
    tagLimelight.uploadGyro(
      drivetrain.getPigeon2().getRotation2d().getDegrees()
    );
    if (groundIntake.isFull() && groundIntake.hasStartedIntaking()) {
      operator.setRumble(RumbleType.kBothRumble, 1);
      driver.setRumble(RumbleType.kBothRumble, 1);
    }
    if (robotSuper.getCurrentSuperState() != CurrentSuperState.INTAKING) {
      operator.setRumble(RumbleType.kBothRumble, 0);
      driver.setRumble(RumbleType.kBothRumble, 0);
    }
    // publisher.set(drivetrain.getGlobalPose());

    if (
      tagLimelight.getLimelightPoseEstimateData() != null &&
      tagLimelight.getLimelightPoseEstimateData().tagCount > 0 &&
      DriverStation.isTeleop()
    ) {
      drivetrain.updateGlobalPoseWithVisionMeasurements(
        tagLimelight.getLimelightPoseEstimateData().pose,
        tagLimelight.getLimelightPoseEstimateData().timestampSeconds
      );
    }
    publisher.set(drivetrain.getGlobalPose());
    // publisherLL.set(tagLimelight.getLimelightPoseEstimateData().pose);
    // if (!DriverStation.getAlliance().isEmpty()) {
    //   if (drivetrain.isInAllianceZone(DriverStation.getAlliance().get())) {
    //     leds.setWantedState(LEDs.WantedState.ZONE_ASSIST);
    //   }
    // }
    SmartDashboard.putNumber(
      "Distance to Hub",
      drivetrain.getDistanceFromHub()
    );
    // Logger.recordOutput(
    //   "Robot/ComponentPoses",
    //   new Pose3d[] { shooter.getShooterPose(), groundIntake.getIntakePose() }
    // );
    SmartDashboard.putNumber(
      "PIGEOn",
      drivetrain.getPigeon2().getRotation2d().getDegrees()
    );
  }

  public void onTeleopInit() {
    robotSuper.setWantedSuperState(WantedSuperState.HOME);
  }

  public void onDisabledInit() {
    leds.setWantedState(WantedState.DISABLED);
    operator.setRumble(RumbleType.kBothRumble, 0);
  }

  public void resetGyro() {
    double headingDeg = DriverStation.getAlliance().get() == Alliance.Blue
      ? 180.0
      : 0.0;
    // double headingDeg = 180;
    drivetrain.getPigeon2().setYaw(headingDeg);
  }
  // public void fixTidalLockInput() {
  //   double controllerInput = operator.getRightX();
  //   if (
  //     Timer.getFPGATimestamp() - lastThetaEdit > 0.5
  //   ) {
  //     if (controllerInput > 0.2) {
  //       lastThetaEdit = Timer.getFPGATimestamp();
  //       dTheta += 5;
  //     } else if (controllerInput < -0.2) {
  //       lastThetaEdit = Timer.getFPGATimestamp();
  //       dTheta -= 5;
  //     }
  //   }
  // }
}

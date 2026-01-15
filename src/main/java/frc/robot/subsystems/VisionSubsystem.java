// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-intake");
  public VisionSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("TX", getIntakeTX());
  }

  public double getIntakeTX() {
    return table.getEntry("tx").getDouble(0.);
  }
  public double getIntakeTY() {
    return table.getEntry("ty").getDouble(0.);
  }
}

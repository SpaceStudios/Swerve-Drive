// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */
  double[] PivotSettings;
  int pivotSetting;
  PivotIO pivotIO;
  public Pivot() {
    PivotSettings = new double[] {0,Math.toRadians(45),Math.toRadians(65)};
    pivotSetting = 0;
    pivotIO = new PivotIO_SIM();
  }

  public void setPivotAngle(double change) {
    if (pivotSetting + change >= 0 && pivotSetting + change <= PivotSettings.length) {
      pivotSetting += change;      
    }
    pivotIO.setPivotAngle(pivotSetting);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pivotIO.PivotUpdate();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gyroscope extends SubsystemBase {

  private static final AHRS _gyro = new AHRS(SerialPort.Port.kMXP);
  private static Gyroscope _instance = null;
  ShuffleboardTab telemetry = Shuffleboard.getTab("Telemetry");

  public static Gyroscope getInstance() {
    if( _instance == null ) {
      _instance = new Gyroscope();
    }
    return _instance;
  }
  /** Creates a new Gyroscope. */
  public Gyroscope() {
    _gyro.calibrate();
    SmartDashboard.putData( "Gyro", _gyro);
  }

public double getPitch() {
  return _gyro.getPitch();
}

public double getRoll() {
    return _gyro.getRoll();
}

public double getYaw() {
    return _gyro.getYaw();
}

public double getDisplacementX() {
    return _gyro.getDisplacementX();
}

public double getDisplacementY() {
    return _gyro.getDisplacementY();
}

public double getDisplacementZ() {
    return _gyro.getDisplacementZ();
}

public Rotation2d getRotation2d() {
    return _gyro.getRotation2d();
}

/*
public Rotation2d getYawRotation() {
  return (Constants.DriveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - getYaw())
          : Rotation2d.fromDegrees(getYaw());
}
*/

public void resetGyro() {
  _gyro.reset();
  System.out.println("Gyro Reset");
}

@Override
public void periodic() {
  // This method will be called once per scheduler run
}

}

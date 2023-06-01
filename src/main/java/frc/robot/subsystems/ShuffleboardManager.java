// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.reset.ResetEncoders;
import frc.robot.commands.reset.ResetGyro;

public class ShuffleboardManager extends SubsystemBase {
  /** Creates a new ShuffleboardManager. */
  private static ShuffleboardManager _instance = null;

  public static ShuffleboardManager getInstance() {
    if( _instance == null ) {
      _instance = new ShuffleboardManager();
    }
    return _instance;
  }

  private final Drivetrain _drive = Drivetrain.getInstance();
  private final Gyroscope _gyro = Gyroscope.getInstance();

  ShuffleboardTab telemetry;

  ShuffleboardTab PID_Drive;

  private GenericEntry frontLeftEncoder;
  private GenericEntry frontRightEncoder;
  private GenericEntry backRightEncoder;
  private GenericEntry backLeftEncoder;
  private GenericEntry gyroPitch;
  private GenericEntry gyroYaw;
  private GenericEntry gyroRoll;
  private GenericEntry xPos;
  private GenericEntry yPos;


  private ShuffleboardManager() {
    shuffleboardInit();
    SmartDashboard.putData("Reset Encoders", new ResetEncoders());
    SmartDashboard.putData("Reset Gyro", new ResetGyro());
    SmartDashboard.putData("Reset All", Commands.run(() -> {
      new ResetGyro().schedule();
      new ResetEncoders().schedule();
    }));
  }

  public void shuffleboardInit() {
    try{
      telemetry = Shuffleboard.getTab("Telemetry");
      frontLeftEncoder = telemetry.add("Front Left Encoder",0).withPosition(0,0).withSize(2,2).withWidget(BuiltInWidgets.kTextView).getEntry();
      frontRightEncoder = telemetry.add("Front Right Encoder",0).withPosition(0,2).withSize(2,2).withWidget(BuiltInWidgets.kTextView).getEntry();
      backLeftEncoder = telemetry.add("Back Left Encoder",0).withPosition(2,0).withSize(2,2).withWidget(BuiltInWidgets.kTextView).getEntry();
      backRightEncoder = telemetry.add("Back Right Encoder",0).withPosition(2,2).withSize(2,2).withWidget(BuiltInWidgets.kTextView).getEntry();

      gyroPitch = telemetry.add("Gyro Pitch", _gyro.getPitch()).withWidget(BuiltInWidgets.kNumberBar).getEntry();
      gyroYaw = telemetry.add("Gyro Yaw", _gyro.getYaw()).withWidget(BuiltInWidgets.kNumberBar).getEntry();
      gyroRoll = telemetry.add("Gyro Roll", _gyro.getRoll()).withWidget(BuiltInWidgets.kNumberBar).getEntry();

      telemetry.add("Reset Gyro", new ResetGyro()).withPosition(7, 0).withSize(1, 1);

      xPos = telemetry.add("X Position", 0)
        .withPosition(4, 1)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
      yPos = telemetry.add("Y Position", 0)
        .withPosition(4, 2)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
                        

    } catch (Exception e) {
      System.out.println( "ShuffleboardManager Problem: " + e );
    }
  }

  public void shuffleboardUpdate() {
    frontLeftEncoder.setDouble(_drive.getFrontLeftVelocity());
    frontRightEncoder.setDouble(_drive.getFrontRightVelocity());
    backRightEncoder.setDouble(_drive.getBackRightVelocity());
    backLeftEncoder.setDouble(_drive.getBackLeftVelocity());
    
    gyroPitch.setDouble(_gyro.getPitch());
    gyroYaw.setDouble(_gyro.getYaw());
    gyroRoll.setDouble(_gyro.getRoll());

    xPos.setDouble(_drive.getPose().getX());
    yPos.setDouble(_drive.getPose().getY());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shuffleboardUpdate();
  }
}

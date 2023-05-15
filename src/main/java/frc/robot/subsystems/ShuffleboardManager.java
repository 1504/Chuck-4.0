// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.reset.ResetEncoders;

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

  ShuffleboardTab telemetry;

  ShuffleboardTab PID_Drive;
  ShuffleboardTab PID_Arm;

  private GenericEntry frontLeftEncoder;
  private GenericEntry frontRightEncoder;
  private GenericEntry backRightEncoder;
  private GenericEntry backLeftEncoder;
  private GenericEntry gyroPitch;
  private GenericEntry gyroYaw;
  private GenericEntry gyroRoll;
  private GenericEntry armPosition;
  private GenericEntry RobotPosition;
  private GenericEntry xPos;
  private GenericEntry yPos;


  private ShuffleboardManager() {
    shuffleboardInit();
    SmartDashboard.putData("Reset Encoders", new ResetEncoders());
  }

  public void shuffleboardInit() {
    try{

    } catch (Exception e) {
      System.out.println( "ShuffleboardManager Problem: " + e );
    }
  }

  public void shuffleboardUpdate() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shuffleboardUpdate();
  }
}

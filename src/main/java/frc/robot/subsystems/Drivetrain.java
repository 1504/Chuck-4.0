// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  
  private CANSparkMax _frontLeft = new CANSparkMax(1,MotorType.kBrushless);
  private CANSparkMax _frontRight = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax _backLeft = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax _backRight = new CANSparkMax(4, MotorType.kBrushless);

  // make 4 new encoder objects
  private final RelativeEncoder leftEncoder = _frontLeft.getEncoder();
  private final RelativeEncoder rightEncoder = _frontRight.getEncoder(); 
  private final RelativeEncoder backLeftEncoder = _backLeft.getEncoder();
  private final RelativeEncoder backRightEncoder = _backRight.getEncoder();

  Translation2d _front_left_location;
  Translation2d _front_right_location;
  Translation2d _back_right_location;
  Translation2d _back_left_location;


  private final HashMap<String, Command> m_eventMap = new HashMap<>();

  Pose2d m_pose;
  
  public Drivetrain() {
    // make 4 new cansparkmax objects

    _frontLeft = new CANSparkMax(DriveConstants.FRONT_LEFT, MotorType.kBrushless);
    _frontRight = new CANSparkMax(DriveConstants.FRONT_RIGHT, MotorType.kBrushless);
    _backRight = new CANSparkMax(DriveConstants.BACK_RIGHT, MotorType.kBrushless);
    _backLeft = new CANSparkMax(DriveConstants.BACK_LEFT, MotorType.kBrushless);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

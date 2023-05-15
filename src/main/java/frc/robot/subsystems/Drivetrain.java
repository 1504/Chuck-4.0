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
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  static private Drivetrain instance = null;
  public static Drivetrain getInstance() {
    if( instance == null ) {
      instance = new Drivetrain();
    }
    return instance;
  }
  /** Creates a new Drivetrain. */
  private final CANSparkMax _frontLeft;
  private final CANSparkMax _frontRight;
  private final CANSparkMax _backLeft;
  private final CANSparkMax _backRight;

  // make 4 new encoder objects
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private final RelativeEncoder backLeftEncoder;
  private final RelativeEncoder backRightEncoder;

  public Drivetrain() {
    // make 4 new cansparkmax objects

    _frontLeft = new CANSparkMax(DriveConstants.FRONT_LEFT, MotorType.kBrushless);
    _frontRight = new CANSparkMax(DriveConstants.FRONT_RIGHT, MotorType.kBrushless);
    _backLeft = new CANSparkMax(DriveConstants.BACK_LEFT, MotorType.kBrushless);
    _backRight = new CANSparkMax(DriveConstants.BACK_RIGHT, MotorType.kBrushless);
    
    _frontLeft.setInverted(false);
    _frontRight.setInverted(false);
    _backLeft.setInverted(true);
    _backRight.setInverted(true);

    leftEncoder = _frontLeft.getEncoder();
    rightEncoder = _frontRight.getEncoder(); 
    backLeftEncoder = _backLeft.getEncoder();
    backRightEncoder = _backRight.getEncoder();

    _frontLeft.setIdleMode(IdleMode.kCoast);
    _frontRight.setIdleMode(IdleMode.kCoast);
    _backLeft.setIdleMode(IdleMode.kCoast);
    _backRight.setIdleMode(IdleMode.kCoast);

  }

  public void resetEncoders() {
    _frontLeft.set(0);
    _frontRight.set(0);
    _backLeft.set(0);
    _backRight.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

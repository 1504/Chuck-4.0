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
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  private static Drivetrain instance = null;
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

  private final PIDController _front_left_pid;
  private final PIDController _front_right_pid;
  private final PIDController _back_right_pid;
  private final PIDController _back_left_pid;

  private boolean rotating = false;

  private MecanumDrive _drive;

  private final MecanumDrivePoseEstimator _poseEstimator;

  private final MecanumDriveOdometry _odometry;

  private final Gyroscope _gyro;
  
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

    double p = -1.8074;
    
    _front_left_pid = new PIDController(p, 0, 0);
    _front_right_pid = new PIDController(p, 0, 0);
    _back_right_pid = new PIDController(p, 0, 0);
    _back_left_pid = new PIDController(p, 0, 0);

    _drive = new MecanumDrive(_frontLeft, _backRight, _frontRight, _backLeft);

    _gyro = new Gyroscope();

    _poseEstimator = new MecanumDrivePoseEstimator(BuildConstants._KINEMATICS, _gyro.getRotation2d(), getWheelPositions(), getPose());
    
    _odometry = new MecanumDriveOdometry(BuildConstants._KINEMATICS, _gyro.getRotation2d(), getWheelPositions());
  
  }
  
  public void cartesianDrive(double xSpeed, double ySpeed, double zRotation) {
    xSpeed *= -1;
    // deadband the inputs
    double zRot = Math.abs(zRotation) < DriveConstants.DEADBAND ? 0 : Math.pow(zRotation, 3);
    double ySpd = Math.abs(ySpeed) < DriveConstants.DEADBAND ? 0 : Math.pow(ySpeed, 3);
    double xSpd = Math.abs(xSpeed) < DriveConstants.DEADBAND ? 0 : Math.pow(xSpeed, 3);
    if (!rotating) {
      _drive.driveCartesian(xSpd, ySpd, zRot);
    }
  }

  public double getFrontRightDistance() {
    return rightEncoder.getPosition() / BuildConstants.GEAR_RATIO * BuildConstants.WHEEL_CIRCUMFERENCE * BuildConstants.INCHES_TO_METERS;
  }

  public double getFrontLeftDistance() {
    return leftEncoder.getPosition() / BuildConstants.GEAR_RATIO * BuildConstants.WHEEL_CIRCUMFERENCE * BuildConstants.INCHES_TO_METERS;
  }

  public double getBackRightDistance() {
    return backRightEncoder.getPosition() / BuildConstants.GEAR_RATIO * BuildConstants.WHEEL_CIRCUMFERENCE * BuildConstants.INCHES_TO_METERS;
  }

  public double getBackLeftDistance() {
    return backLeftEncoder.getPosition() / BuildConstants.GEAR_RATIO * BuildConstants.WHEEL_CIRCUMFERENCE * BuildConstants.INCHES_TO_METERS;
  }





  public double getFrontLeftVelocity() {
    return leftEncoder.getVelocity() / BuildConstants.GEAR_RATIO * BuildConstants.WHEEL_CIRCUMFERENCE / 60 * BuildConstants.INCHES_TO_METERS;
  }

  public double getFrontRightVelocity() {
    return rightEncoder.getVelocity() / BuildConstants.GEAR_RATIO * BuildConstants.WHEEL_CIRCUMFERENCE / 60 * BuildConstants.INCHES_TO_METERS;
  }

  public double getBackRightVelocity() {
    return backRightEncoder.getVelocity() / BuildConstants.GEAR_RATIO * BuildConstants.WHEEL_CIRCUMFERENCE / 60 * BuildConstants.INCHES_TO_METERS;
  }

  public double getBackLeftVelocity() {
    return backLeftEncoder.getVelocity() / BuildConstants.GEAR_RATIO * BuildConstants.WHEEL_CIRCUMFERENCE / 60 * BuildConstants.INCHES_TO_METERS;
  }


  public void resetEncoders() {
    _frontLeft.set(0);
    _frontRight.set(0);
    _backLeft.set(0);
    _backRight.set(0);
  }

  public Pose2d getPose() {
    if (Constants.AutoConstants.USE_VISION_ASSIST) {
      return _poseEstimator.getEstimatedPosition();
    } else {
      return _odometry.getPoseMeters();
    }
  }

  private void setWheelSpeeds(double frontLeft, double frontRight, double backLeft, double backRight) {
    _front_left_pid.setSetpoint(frontLeft);
    _front_right_pid.setSetpoint(frontRight);
    _back_left_pid.setSetpoint(backLeft);
    _back_right_pid.setSetpoint(backRight);
    _frontLeft.setVoltage(_front_left_pid.calculate(getFrontLeftVelocity()));
    _frontRight.setVoltage(_front_right_pid.calculate(getFrontRightVelocity()));
    _backLeft.setVoltage(_back_left_pid.calculate(getBackLeftVelocity()));
    _backRight.setVoltage(_back_right_pid.calculate(getBackRightVelocity()));
  }

  public void resetOdometry(Pose2d pose) {
    MecanumDriveWheelPositions positions = getWheelPositions();
    if (!Constants.AutoConstants.USE_VISION_ASSIST) {
      _poseEstimator.resetPosition(new Rotation2d(_gyro.getYaw()), positions, pose);
    } else {
      _odometry.resetPosition(new Rotation2d(_gyro.getYaw()), positions, pose);
    }
  }

  private MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(
      getFrontLeftDistance(),
      getFrontRightDistance(),
      getBackLeftDistance(),
      getBackRightDistance()
    );
  }

  public void setOutputWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
    double frontLeft = wheelSpeeds.frontLeftMetersPerSecond;
    double frontRight = wheelSpeeds.frontRightMetersPerSecond;
    double backLeft = wheelSpeeds.rearLeftMetersPerSecond;
    double backRight = wheelSpeeds.rearRightMetersPerSecond;
    setWheelSpeeds(frontLeft, frontRight, backLeft, backRight);
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(new InstantCommand(() -> {
      // Reset odometry for the first path you run during auto
      if (isFirstPath) {
        this.resetOdometry(new Pose2d());
      }
    }), new PPMecanumControllerCommand(traj, this::getPose, // Pose supplier
            BuildConstants._KINEMATICS, // Kinematics object
            new PIDController(0.00, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(0.00, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(0.00, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            Constants.AutoConstants.AUTO_MAX_SPEED_METERS_PER_SECOND,  // Max wheel velocity meters per second
            this::setOutputWheelSpeeds, // MecanumDriveWheelSpeeds consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
    ));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

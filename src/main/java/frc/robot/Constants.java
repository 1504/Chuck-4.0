// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {

    public static final boolean invertGyro = true;
    
    //fix these
    public static final int FRONT_LEFT = 0;
    public static final int FRONT_RIGHT = 0;
    public static final int BACK_LEFT = 0;
    public static final int BACK_RIGHT = 0;

    public static final double DEADBAND = 0;
  }

  public static class BuildConstants { 

    public static final double WHEEL_CIRCUMFERENCE = 6 * Math.PI;
    public static final double INCHES_TO_METERS = 0.0254;
    public static final double INCHES_PER_REVOLUTION = WHEEL_CIRCUMFERENCE / INCHES_TO_METERS;
    public static final double WHEEL_TO_CENTER_SIDE_INCHES = 0.26 / INCHES_TO_METERS;
    public static final double WHEEL_TO_CENTER_FRONT_INCHES = 0.3175 / INCHES_TO_METERS;
    public static final double GEAR_RATIO = 12;

    
    // Wheel positions
    public static final Translation2d _FRONT_LEFT_LOCATION = new Translation2d(
        WHEEL_TO_CENTER_SIDE_INCHES * INCHES_TO_METERS, WHEEL_TO_CENTER_FRONT_INCHES * INCHES_TO_METERS);
    public static final Translation2d _FRONT_RIGHT_LOCATION = new Translation2d(
        -WHEEL_TO_CENTER_SIDE_INCHES * INCHES_TO_METERS, WHEEL_TO_CENTER_FRONT_INCHES * INCHES_TO_METERS);
    public static final Translation2d _BACK_LEFT_LOCATION = new Translation2d(
        WHEEL_TO_CENTER_SIDE_INCHES * INCHES_TO_METERS, -WHEEL_TO_CENTER_FRONT_INCHES * INCHES_TO_METERS);
    public static final Translation2d _BACK_RIGHT_LOCATION = new Translation2d(
        -WHEEL_TO_CENTER_SIDE_INCHES * INCHES_TO_METERS, -WHEEL_TO_CENTER_FRONT_INCHES * INCHES_TO_METERS);

    public static final MecanumDriveKinematics _KINEMATICS = new MecanumDriveKinematics(
        _FRONT_LEFT_LOCATION, _FRONT_RIGHT_LOCATION, _BACK_LEFT_LOCATION, _BACK_RIGHT_LOCATION);

  }

  public static class AutoConstants { 
    public static final double AUTO_MAX_SPEED_METERS_PER_SECOND = 1;

    public static final boolean USE_VISION_ASSIST = false;
  }
}

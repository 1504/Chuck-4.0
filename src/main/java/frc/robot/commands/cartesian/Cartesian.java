// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.cartesian;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Cartesian extends CommandBase {
  /** Creates a new Cartesian. */

  private Drivetrain _instance = Drivetrain.getInstance(); 

  private final DoubleSupplier _ySpeed;
  private final DoubleSupplier _xSpeed;
  private final DoubleSupplier _zRotation;
  //private final Drivetrain m_drivetrain = Drivetrain.getInstance();

  public Cartesian(DoubleSupplier forwardSpeed, DoubleSupplier horizontalSpeed, DoubleSupplier rotationSpeed ) {
    // Use addRequirements() here to declare subsystem dependencies.

    _ySpeed = horizontalSpeed;
    _xSpeed = forwardSpeed;
    _zRotation = rotationSpeed;

    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _instance.cartesianDrive(_xSpeed.getAsDouble(), _ySpeed.getAsDouble(), _zRotation.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

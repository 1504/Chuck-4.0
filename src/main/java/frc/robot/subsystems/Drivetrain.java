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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final CANSparkMax _frontLeft = new CANSparkMax(1,MotorType.kBrushless);
  private final CANSparkMax _frontRight = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax _backLeft = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax _backRight = new CANSparkMax(4, MotorType.kBrushless);

    // make 4 new encoder objects
    private final RelativeEncoder leftEncoder = _frontLeft.getEncoder();
    private final RelativeEncoder rightEncoder = _frontRight.getEncoder(); 
    private final RelativeEncoder backLeftEncoder = _backLeft.getEncoder();
    private final RelativeEncoder backRightEncoder = _backRight.getEncoder();

  public Drivetrain() {
    // make 4 new cansparkmax objects
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

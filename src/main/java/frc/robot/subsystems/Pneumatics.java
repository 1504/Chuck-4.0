// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {

  private static Pneumatics _instance = null;

  public static Pneumatics getInstance() {
    if( _instance == null ) {
      _instance = new Pneumatics();
    }
    return _instance;
  }
  private static Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  private final Solenoid _solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  /** Creates a new Pneumatics. */
  public Pneumatics() {
    m_compressor.enableDigital();
  }

  public void enableSolenoid() { 
    _solenoid.set(true);
  }

  public void disableSolenoid() {
    _solenoid.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.cartesian.Cartesian;
import frc.robot.commands.pneumatics.DisableSolenoid;
import frc.robot.commands.pneumatics.EnableSolenoid;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.ShuffleboardManager;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //subsystems
  private final Drivetrain m_drivetrain = Drivetrain.getInstance();
  private final ShuffleboardManager m_shuffleboardManager = ShuffleboardManager.getInstance();
  private final Pneumatics m_pneumatics = Pneumatics.getInstance();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick m_joystick1 = new Joystick(1);
  private final Joystick m_joystick2 = new Joystick(2);


  private final SendableChooser<CommandBase> m_autoChooser = new SendableChooser<>();
  private final List<List<PathPlannerTrajectory>> m_testPaths = new ArrayList<>();

  public static final HashMap<String, Command> m_eventMap = new HashMap<>();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_drivetrain.setDefaultCommand(new Cartesian(() -> m_joystick1.getX(), () ->m_joystick1.getY(), () ->m_joystick2.getY()));
    new JoystickButton(m_joystick1, 1).whileTrue(new EnableSolenoid());
    new JoystickButton(m_joystick1, 2).whileTrue(new DisableSolenoid());
    }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //if (m_autoChooser.getSelected() == null) {
      return new PrintCommand("No Auton Selected");
    //}
  
  }
}

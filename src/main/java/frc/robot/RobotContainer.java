// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.BaseCommand;
import frc.robot.commands.EndGameCommand;
import frc.robot.commands.VisionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  // Chooser
  private final SendableChooser<Command> autoChooser;
  public final ArmCommand m_armcommand = new ArmCommand(m_armSubsystem);
  public final BaseCommand m_baseCommand = new BaseCommand(m_swerveSubsystem);
  public final EndGameCommand m_endgameCommand = new EndGameCommand(m_climbSubsystem);
  public final VisionCommand m_visionCommand = new VisionCommand(m_swerveSubsystem, m_armSubsystem, m_visionSubsystem);

  public static final XboxController baseJoystick = new XboxController(0);
  public static final XboxController armJoystick = new XboxController(1);

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto mode", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    new Servo(0).getAngle();
  }

  public Command getAutonomousCommand() {
    return null;
  }
}

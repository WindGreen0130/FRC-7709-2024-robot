// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import static frc.robot.RobotContainer.*;

public class EndGameCommand extends Command {
  /** Creates a new EndGameCommand. */
  private final ClimbSubsystem climbsubsystem;
  public EndGameCommand(ClimbSubsystem _climbSubsystem) {
    this.climbsubsystem = _climbSubsystem;
    addRequirements(climbsubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbsubsystem.climbfalse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbsubsystem.climb(armJoystick.getRawAxis(1)*0.3, armJoystick.getRawAxis(5)*0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

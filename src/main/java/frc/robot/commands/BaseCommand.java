// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import static frc.robot.RobotContainer.*;

public class BaseCommand extends Command {
  /** Creates a new BaseCommand. */
  private final SwerveSubsystem swerveSubsystem;

  private double xspeed;
  private double ySpeed;
  private double zSpeed;
  private final double deadValue = 0.1;

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(4);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(4);
  public BaseCommand(SwerveSubsystem _swerveSubsystem) {
    this.swerveSubsystem = _swerveSubsystem;
    addRequirements(_swerveSubsystem);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xspeed = xLimiter.calculate(Constants.SwerveConstants.joysickValue(baseJoystick.getRawAxis(1), deadValue)*0.6);
    ySpeed = yLimiter.calculate(Constants.SwerveConstants.joysickValue(baseJoystick.getRawAxis(0), deadValue)*0.6);
    zSpeed = Constants.SwerveConstants.joysickValue(baseJoystick.getRawAxis(4), deadValue);
    swerveSubsystem.drive(xspeed, ySpeed, zSpeed, true);
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionCommand extends Command {
  /** Creates a new VisionCommand. */
  private final SwerveSubsystem swerveSubsystem;
  private final ArmSubsystem armSubsystem;
  private final VisionSubsystem visionSubsystem;
  private double xSpeed;
  private double ySpeed;
  private double zSpeed;
  public VisionCommand(SwerveSubsystem _swerveSubsystem, ArmSubsystem _armSubsystem, VisionSubsystem _visionSubsystem) {
    this.swerveSubsystem = _swerveSubsystem;
    this.armSubsystem = _armSubsystem;
    this.visionSubsystem = _visionSubsystem;
    addRequirements(swerveSubsystem, armSubsystem, visionSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xSpeed = visionSubsystem.xMovePIDOutput;
    ySpeed = visionSubsystem.yMovePIDOutput;
    zSpeed = visionSubsystem.turnPIDOutput;
    swerveSubsystem.drive(xSpeed, ySpeed, zSpeed, false);
    armSubsystem.getObjectDistance(visionSubsystem.botXValue);
    armSubsystem.armPIDCalculate(armSubsystem.armAimSetpoint);
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

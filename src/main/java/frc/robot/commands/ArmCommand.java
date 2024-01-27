// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {
  /** Creates a new ArmCommand. */
  private final ArmSubsystem armSubsystem;
  public ArmCommand(ArmSubsystem _armsubsystem) {
    this.armSubsystem = _armsubsystem;
    addRequirements(armSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  int k;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for(int i = 1;i<=10;i++){
      if(RobotContainer.armJoystick.getRawButton(i)){
        k = i;
      }
    }
    switch (k) {
      case 1:
        armSubsystem.armPIDCalculate(Constants.ArmConstants.armOrigin);
        break;
      case 2:
        armSubsystem.shoot();
        break;
      case 3:
        armSubsystem.armPIDCalculate(Constants.ArmConstants.armTaking);
        break;
      case 4:
        armSubsystem.take();
      default:
        armSubsystem.armPIDCalculate(armSubsystem.armAimSetpoint);
        break;
    }
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

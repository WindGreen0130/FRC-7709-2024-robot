// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final CANSparkMax armMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax armMotor2 = new CANSparkMax(2, MotorType.kBrushless);

  private final DigitalInput armDigitalInput = new DigitalInput(0);

  private final ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0, 0);
  private final PIDController armPID = new PIDController(0, 0, 0);
  
  private double armFeedforwardOutput;
  private double armPIDOutput;


  private double armMoveOutput;


  public ArmSubsystem() {
    armMotor2.follow(armMotor1);

    armMotor1.restoreFactoryDefaults();
    armMotor2.restoreFactoryDefaults();

    armMotor1.setIdleMode(IdleMode.kBrake);
    armMotor2.setIdleMode(IdleMode.kBrake);

    armMotor1.setInverted(false);
    armMotor2.setInverted(false);

    armMotor1.burnFlash();
    armMotor2.burnFlash();
  }

  @Override
  public void periodic() {
    armFeedforwardOutput = armFeedforward.calculate(armMoveOutput, armFeedforwardOutput);
    // This method will be called once per scheduler run
  }
}

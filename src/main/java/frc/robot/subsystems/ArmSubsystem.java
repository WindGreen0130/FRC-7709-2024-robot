// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ApriltagConstants.*;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final CANSparkMax armMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax armMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax shooterMotor1 = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax shooterMotor2 = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
  
  private final DigitalInput armDigitalInput = new DigitalInput(0);

  private final ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0, 0);
  private final PIDController armPID = new PIDController(0, 0, 0);
  public static double aimedPoint;
  private double armFeedforwardOutput;
  private double armPIDOutput;


  private double armMoveOutput;


  public ArmSubsystem() {
    armMotor2.follow(armMotor1);
    Math.atan(6);
    armMotor1.restoreFactoryDefaults();
    armMotor2.restoreFactoryDefaults();
    intakeMotor.restoreFactoryDefaults();
    shooterMotor1.restoreFactoryDefaults();
    shooterMotor2.restoreFactoryDefaults();

    armMotor1.setIdleMode(IdleMode.kBrake);
    armMotor2.setIdleMode(IdleMode.kBrake);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    shooterMotor1.setIdleMode(IdleMode.kBrake);
    shooterMotor2.setIdleMode(IdleMode.kBrake);

    armMotor1.setInverted(false);
    armMotor2.setInverted(false);
    intakeMotor.setInverted(false);
    shooterMotor1.setInverted(false);
    shooterMotor2.setInverted(false);

    armMotor1.burnFlash();
    armMotor2.burnFlash();
    intakeMotor.burnFlash();
    shooterMotor1.burnFlash();
    shooterMotor2.burnFlash();
  }
  public void take(){
    intakeMotor.set(0.5);
  }
  public void shoot(){
    intakeMotor.set(0.5);
    shooterMotor1.set(0.5);
    shooterMotor2.set(0.5);
  }
  public void stop(){
    intakeMotor.set(0);
    shooterMotor1.set(0);
    shooterMotor2.set(0);
  }
  public void ArmPIDCalculate(double setpoint){
    armPID.calculate(armPosition,setpoint);
  }
  @Override
  public void periodic() {
    armFeedforwardOutput = armFeedforward.calculate(armMoveOutput, armFeedforwardOutput);
    // This method will be called once per scheduler run
  }
}

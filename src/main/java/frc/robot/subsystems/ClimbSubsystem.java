// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  //Motor
  private final CANSparkMax climberMotor1 = new CANSparkMax(5, MotorType.kBrushed);
  private final CANSparkMax climberMotor2 = new CANSparkMax(6, MotorType.kBrushed);
  //Servo
  private final Servo holdingMotor = new Servo(0);

  public static boolean climb = false;

  public ClimbSubsystem() {
    climberMotor1.restoreFactoryDefaults();
    climberMotor2.restoreFactoryDefaults();

    climberMotor1.setInverted(true);
    climberMotor2.setInverted(true);

    climberMotor1.burnFlash();
    climberMotor2.burnFlash();
  }

  public void climb(double leftSpeed, double rightSpeed){
    climberMotor1.set(leftSpeed);
    climberMotor2.set(rightSpeed);
  }

  public void climbDone(){
    holdingMotor.setAngle(90);
    climb = true;
  }

  public void climbfalse(){
    holdingMotor.setAngle(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

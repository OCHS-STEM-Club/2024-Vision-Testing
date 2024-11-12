// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  private CANSparkMax armMotorLeft;
  private CANSparkMax armMotorRight;
  private RelativeEncoder armEncoderLeft;
  private RelativeEncoder armEncoderRight;
  private DigitalInput lowerHardStop;
  private DigitalInput upperHardStop;
  private boolean atshootervalue;

  private SparkPIDController m_pidController;
  private AbsoluteEncoder m_encoder;

  private double speed;
  private double value;
  private double armValue;
  private boolean turningMode;


  public ArmSubsystem() {
    armMotorLeft = new CANSparkMax(Constants.ArmConstants.kArmMotorLeftID, MotorType.kBrushless);
    armMotorRight = new CANSparkMax(Constants.ArmConstants.kArmMotorRightID, MotorType.kBrushless);
    armMotorLeft.setIdleMode(IdleMode.kBrake);
    armMotorRight.setIdleMode(IdleMode.kBrake);
    armMotorLeft.setInverted(false);
    armMotorRight.setInverted(false);

    armMotorRight.setSmartCurrentLimit(30,20);
    armMotorLeft.setSmartCurrentLimit(30,20);

    m_encoder = armMotorRight.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_encoder.setPositionConversionFactor(360);
    m_encoder.setZeroOffset(ArmConstants.kEncoderZeroOffset);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

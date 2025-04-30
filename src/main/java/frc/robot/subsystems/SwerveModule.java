// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import frc.robot.Constants.SwerveConstants.MotorLocation;

public class SwerveModule{
  /** Creates a new ExampleSubsystem. */
  //Motors
  private final SparkMax m_driveMotor;
  private final SparkMax m_turnMotor;

  //Encoders
  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turnEncoder;
  private final CANcoder m_absoluteEncoder;

  //PID
  private final SparkClosedLoopController m_driveLoopController;
  private final SparkClosedLoopController m_turnLoopController;

  private final MotorLocation m_motorLocation;
  private final double m_encoderInversion;
  private final double m_absoluteEncoderReversed;
  private final double m_absoluteEncoderOffset;
  private double m_chassisAngularOffset = 0;

  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public SwerveModule(int p_driveID, int p_turnID, int p_absID, double p_offset, boolean p_encoderInverted, boolean p_absInverted, 
                      double p_absOffset, SparkMaxConfig p_driveConfig, SparkMaxConfig p_turnConfig, MotorLocation p_motorLocation) {
    
    m_driveMotor = new SparkMax(p_driveID, MotorType.kBrushless);
    m_turnMotor = new SparkMax(p_turnID, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turnEncoder = m_turnMotor.getEncoder();
    m_absoluteEncoder = new CANcoder(p_absID);

    m_driveLoopController = m_driveMotor.getClosedLoopController();
    m_turnLoopController = m_turnMotor.getClosedLoopController();

    m_motorLocation = p_motorLocation;

    m_driveMotor.configure(p_driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turnMotor.configure(p_turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); //change?

    m_chassisAngularOffset = p_offset;
    m_absoluteEncoderOffset = p_absOffset;
    m_desiredState.angle = new Rotation2d(m_turnEncoder.getPosition());
    
    m_encoderInversion = (p_encoderInverted) ? -1.0 : 1.0;
    m_absoluteEncoderReversed = (p_absInverted) ? -1.0 : 1.0;

    resetEncoders();
  }

  public double getAbsoluteEncoderRad() {
    return m_absoluteEncoder.getAbsolutePosition().getValue().in(Units.Radians) - m_absoluteEncoderOffset * m_absoluteEncoderReversed;
  }

  public SwerveModuleState getDesiSwerveModuleState() {
    return m_desiredState;
  }

  //in meters
  public double getDrivePositon() {
    return m_driveEncoder.getPosition();
  }

  public double getTurnPosition() {
    return m_turnEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return m_driveEncoder.getVelocity();
  }

  //in radians
  public double getTurnVelocity() {
    return m_encoderInversion * m_turnEncoder.getVelocity();
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveVelocity(), new Rotation2d(getTurnPosition()));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
  }

  public void setDesiredState(SwerveModuleState p_desiredState) { //help
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = m_desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = m_desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    correctedDesiredState.optimize(new Rotation2d(getAbsoluteEncoderRad())); 

    m_driveLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turnLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = p_desiredState;
  }

  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turnEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public void stop() {
    m_driveMotor.set(0);
    m_turnMotor.set(0);
  }
}

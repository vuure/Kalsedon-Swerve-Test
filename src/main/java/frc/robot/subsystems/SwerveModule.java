// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {

  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Math.PI;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; 

  private final PWMSparkMax driveMotor;
  private final PWMSparkMax turningMotor;

  private final Encoder driveEncoder;
  private final Encoder turningEncoder;

  private final PIDController drivePIDController = new PIDController(1, 0, 0);

  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int driveEncoderChannelA,
      int driveEncoderChannelB,
      int turningEncoderChannelA,
      int turningEncoderChannelB) {

    driveMotor = new PWMSparkMax(driveMotorChannel);
    turningMotor = new PWMSparkMax(turningMotorChannel);

    driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);
    turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);

    driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getRate(), new Rotation2d(turningEncoder.getDistance()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getDistance(), new Rotation2d(turningEncoder.getDistance()));
  }


  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(turningEncoder.getDistance());

    desiredState.optimize(encoderRotation);

    desiredState.cosineScale(encoderRotation);

    final double driveOutput =
        drivePIDController.calculate(driveEncoder.getRate(), desiredState.speedMetersPerSecond);

    final double turnOutput =
        m_turningPIDController.calculate(
            turningEncoder.getDistance(), desiredState.angle.getRadians());

    driveMotor.setVoltage(driveOutput);
    turningMotor.setVoltage(turnOutput);
  }

  @Override
  public void periodic() {
  }
}

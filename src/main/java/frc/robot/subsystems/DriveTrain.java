package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
        // Add motor controllers once they are decided.

      private final CANSparkMax DRIVE_LEFT_FRONT;
      private final CANSparkMax DRIVE_RIGHT_FRONT;
      private final CANSparkMax DRIVE_LEFT_BACK;
      private final CANSparkMax DRIVE_RIGHT_BACK;
      private final ADXRS450_Gyro gyro;

      public static RelativeEncoder left_front_encoder;
      public static RelativeEncoder left_rear_encoder;
      public static RelativeEncoder right_front_encoder;
      public static RelativeEncoder right_rear_encoder;
      private final DifferentialDriveOdometry odometry;

      private final DifferentialDrive my_robot;

  public DriveTrain() {
    DRIVE_LEFT_FRONT = new CANSparkMax(Constants.LEFT_FRONT_MOTOR, MotorType.kBrushless);
    DRIVE_RIGHT_FRONT = new CANSparkMax(Constants.RIGHT_FRONT_MOTOR, MotorType.kBrushless);
    DRIVE_LEFT_BACK = new CANSparkMax(Constants.LEFT_BACK_MOTOR, MotorType.kBrushless);
    DRIVE_RIGHT_BACK = new CANSparkMax(Constants.RIGHT_BACK_MOTOR, MotorType.kBrushless);

    DRIVE_LEFT_FRONT.clearFaults();
    DRIVE_RIGHT_FRONT.clearFaults();
    DRIVE_LEFT_BACK.clearFaults();
    DRIVE_RIGHT_BACK.clearFaults();


    DRIVE_LEFT_FRONT.restoreFactoryDefaults();
    DRIVE_RIGHT_FRONT.restoreFactoryDefaults();
    DRIVE_LEFT_BACK.restoreFactoryDefaults();
    DRIVE_RIGHT_BACK.restoreFactoryDefaults();

    DRIVE_LEFT_FRONT.setIdleMode(IdleMode.kCoast);
    DRIVE_RIGHT_BACK.setIdleMode(IdleMode.kCoast);
    DRIVE_RIGHT_FRONT.setIdleMode(IdleMode.kCoast);
    DRIVE_LEFT_BACK.setIdleMode(IdleMode.kCoast);



    DRIVE_LEFT_BACK.follow(DRIVE_LEFT_FRONT);
    DRIVE_RIGHT_BACK.follow(DRIVE_RIGHT_FRONT);

    // Encoders
    left_front_encoder = DRIVE_LEFT_FRONT.getEncoder();
    left_rear_encoder = DRIVE_LEFT_BACK.getEncoder();
    right_front_encoder = DRIVE_RIGHT_FRONT.getEncoder();
    right_rear_encoder = DRIVE_RIGHT_BACK.getEncoder();

    
    my_robot = new DifferentialDrive(DRIVE_LEFT_FRONT, DRIVE_RIGHT_FRONT);
    gyro = new ADXRS450_Gyro();

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    //hog rida 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      SmartDashboard.putNumber("Left RPM", left_front_encoder.getVelocity());
      SmartDashboard.putNumber("Right RPM", right_front_encoder.getVelocity());

      double heading = getHeading();
      double leftDist = getLeftDistance();
      double rightDist = getRightDistance();

      odometry.update(
              Rotation2d.fromDegrees(heading),
              leftDist,
              rightDist
      );
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public DifferentialDriveWheelSpeeds getSpeeds() {
      return new DifferentialDriveWheelSpeeds(
              (left_front_encoder.getVelocity() / 7.29 * 2 * Math.PI * Units.inchesToMeters(6.0) / 60),
              (right_rear_encoder.getVelocity() / 7.29 * 2 * Math.PI * Units.inchesToMeters(6.0) / 60)
      );
  }

  public synchronized void manualDrive(double leftVal, double rightVal) {
    DRIVE_LEFT_FRONT.set(leftVal);
    DRIVE_RIGHT_FRONT.set(rightVal);
  }

    public synchronized void tankDriveVolts(double leftVolts, double rightVolts) {
        DRIVE_LEFT_FRONT.setVoltage(-leftVolts);
        DRIVE_RIGHT_BACK.setVoltage(-rightVolts);
        my_robot.feed();
    }

    public synchronized void arcadeDrive(double xVal, double yVal) {
        my_robot.tankDrive(xVal, yVal);
    }

    public synchronized void resetOdometry(Pose2d pose) {
        zeroHeading();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360) * (true ? -1.0 : 1.0);
    }

    public double getLeftDistance() {
        return -left_front_encoder.getPosition() * (((Math.PI * Units.inchesToMeters(6)) / 2048) / 10.71);
        //return -m_leftFront.getSelectedSensorPosition(0) * encoderConstant;
    }

    public double getRightDistance() {
        return -right_front_encoder.getPosition() * (((Math.PI * Units.inchesToMeters(6)) / 2048) / 10.71);
        //return -m_rightFront.getSelectedSensorPosition(0) * encoderConstant;
    }
    public synchronized void zeroHeading() {
        gyro.reset();
    }
}

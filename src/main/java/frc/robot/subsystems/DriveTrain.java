package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
        // Add motor controllers once they are decided.

      private final CANSparkMax DRIVE_LEFT_FRONT;
      private final CANSparkMax DRIVE_RIGHT_FRONT;
      private final CANSparkMax DRIVE_LEFT_BACK;
      private final CANSparkMax DRIVE_RIGHT_BACK;

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


    DRIVE_LEFT_BACK.follow(DRIVE_LEFT_FRONT);
    DRIVE_RIGHT_BACK.follow(DRIVE_RIGHT_FRONT);

    
    my_robot = new DifferentialDrive(DRIVE_LEFT_FRONT, DRIVE_RIGHT_FRONT);
    //hog rida 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public synchronized void manualDrive(double leftVal, double rightVal) {
    DRIVE_LEFT_FRONT.set(leftVal);
    DRIVE_RIGHT_FRONT.set(rightVal);
  }

    public synchronized void arcadeDrive(double xVal, double yVal) {
        my_robot.tankDrive(xVal, yVal);
    }
}

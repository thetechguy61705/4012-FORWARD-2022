// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {

    public static class AutonomousVariables {
        public static final double kS = 0.23107;
        public static final double kV = 0.13003;
        public static final double kA = 0.02345;
        public static final double kP = 0.17751;


        public static final double kPDriveVel = 8.5;

        public static final double kMaxSpeed = 1;
        public static final double kMaxAcceleration = .6;

        public static final double kRameseteB = 2;
        public static final double kRameseteZeta = .7;
        public static final double trackWidthMeters = .5969;

        public static final double maxVoltage = 5;

        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(trackWidthMeters);
    }

    // Joysticks
    public static int LEFT_STICK = 0;
    public static int RIGHT_STICK = 1;


    //DriveTrain
    public static int LEFT_FRONT_MOTOR = 1;
    public static int RIGHT_FRONT_MOTOR = 3;
    public static int LEFT_BACK_MOTOR = 2;
    public static int RIGHT_BACK_MOTOR = 4;


    // Shooter
    public static int TOP_SHOOTER_MOTOR = 5;
    public static int BOTTOM_SHOOTER_MOTOR = 6;

    // Intake
    /**
     * The assigned motor is a 775 pro motor, with a Talon SRX Motor Controller
     */
    public static int INTAKE_MOTOR_PORT = 7;
  }

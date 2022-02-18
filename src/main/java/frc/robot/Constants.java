// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

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

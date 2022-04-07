// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

import java.io.IOError;
import java.io.IOException;
import java.nio.file.Path;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private DriveTrain driveTrain = new DriveTrain();
  private Limelight limelight = new Limelight();
  private Intake intake = new Intake();
  private Shooter shooter = new Shooter();
  private Joystick stickL = new Joystick(Constants.LEFT_STICK);
  private Joystick stickR = new Joystick(Constants.RIGHT_STICK);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configDefaultCommands();
  }

  private void configureButtonBindings() {

    JoystickButton shootBalls = new JoystickButton(stickR, 1);
    JoystickButton TargetTrack = new JoystickButton(stickR, 2);
    JoystickButton putBallsIn = new JoystickButton(stickR, 3);
    JoystickButton runShooterBottom = new JoystickButton(stickR, 8);
    JoystickButton runShooterTop = new JoystickButton(stickR, 9);
    JoystickButton visionProcessor = new JoystickButton(stickL, 8);
    JoystickButton driverCamera = new JoystickButton(stickL, 9);


    putBallsIn.whileHeld(new IntakeManipulator(intake, () -> -stickL.getRawAxis(3)));
    TargetTrack.whileHeld(new TrackTarget(driveTrain, limelight));
    shootBalls.whileHeld(new ShootBalls(shooter, () -> -stickR.getRawAxis(3)));
    runShooterBottom.whileHeld(new bottomShooter(shooter));
    runShooterTop.whileHeld(new topShooter(shooter, () -> -stickR.getRawAxis(3)));
    visionProcessor.whenPressed(new CameraMode0(limelight));
    driverCamera.whenPressed(new CameraMode1(limelight));

  }
  private void configDefaultCommands() {
    driveTrain.setDefaultCommand(new TankDrive(driveTrain,
            () -> -stickL.getY(),
            () -> stickR.getY()));

    limelight.setDefaultCommand(new SetVision(limelight));


  }


  public Command getTestCommand() {
    
    // return new ParallelCommandGroup(
    //   new TankDrive(driveTrain, () -> 0.75, () -> 0.75),
    //   new IntakeManipulator(intake, () -> -0.75),
    //         new ShootBalls(shooter, () -> 0.75)
    // )
    //         .withTimeout(10);
    return null;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(
                            Constants.AutonomousVariables.kS,
                            Constants.AutonomousVariables.kV,
                            Constants.AutonomousVariables.kA
                    ),
                    Constants.AutonomousVariables.kDriveKinematics,
                    Constants.AutonomousVariables.maxVoltage
            );

    TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutonomousVariables.kMaxSpeed,
                    Constants.AutonomousVariables.kMaxAcceleration
            ).setKinematics(Constants.AutonomousVariables.kDriveKinematics)
                    .addConstraint(autoVoltageConstraint);


    String redTrajectoryToShoot = "paths/Red Shoot.wpilib.json";
    String BlueTrajectoryToShoot = "paths/Blue Side Shoot.wpilib.json";
    String redIdleTrajectory = "paths/Red Shoot to idle.wpilib.json";
    String BlueIdleTrajectory = "paths/Blue Shoot to Idle.wpilib.json";

    Trajectory shoot = new Trajectory();
    Trajectory idle = new Trajectory();
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      try {
        Path ShootTrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(redTrajectoryToShoot);
        Path IdleTrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(redIdleTrajectory);
        shoot = TrajectoryUtil.fromPathweaverJson(ShootTrajectoryPath);
        idle = TrajectoryUtil.fromPathweaverJson(IdleTrajectoryPath);
      } catch(IOError | IOException error) {
        DriverStation.reportError("I couldn't open the trajectory: " + redTrajectoryToShoot, error.getStackTrace());
      }
    } else {
      try {
        Path ShootTrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(BlueTrajectoryToShoot);
        Path IdleTrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(BlueIdleTrajectory);
        shoot = TrajectoryUtil.fromPathweaverJson(ShootTrajectoryPath);
        idle = TrajectoryUtil.fromPathweaverJson(IdleTrajectoryPath);
      } catch(IOError | IOException error) {
        DriverStation.reportError("I couldn't open the trajectory: " + BlueTrajectoryToShoot, error.getStackTrace());
      }
    }

    RamseteCommand ramseteCommand1 =
            new RamseteCommand(
                    shoot,
                    driveTrain::getPose,
                    new RamseteController(Constants.AutonomousVariables.kRameseteB, Constants.AutonomousVariables.kRameseteZeta),
                    new SimpleMotorFeedforward(
                            Constants.AutonomousVariables.kS,
                            Constants.AutonomousVariables.kV,
                            Constants.AutonomousVariables.kA),
                    Constants.AutonomousVariables.kDriveKinematics,
                    driveTrain::getSpeeds,
                    new PIDController(Constants.AutonomousVariables.kPDriveVel, 0, 0),
                    new PIDController(Constants.AutonomousVariables.kPDriveVel, 0, 0),
                    // RamseteCommand passes volts to the callback
                    driveTrain::tankDriveVolts,
                    driveTrain);
    RamseteCommand ramseteCommand2 =
            new RamseteCommand(
                    idle,
                    driveTrain::getPose,
                    new RamseteController(Constants.AutonomousVariables.kRameseteB, Constants.AutonomousVariables.kRameseteZeta),
                    new SimpleMotorFeedforward(
                            Constants.AutonomousVariables.kS,
                            Constants.AutonomousVariables.kV,
                            Constants.AutonomousVariables.kA),
                    Constants.AutonomousVariables.kDriveKinematics,
                    driveTrain::getSpeeds,
                    new PIDController(Constants.AutonomousVariables.kPDriveVel, 0, 0),
                    new PIDController(Constants.AutonomousVariables.kPDriveVel, 0, 0),
                    // RamseteCommand passes volts to the callback
                    driveTrain::tankDriveVolts,
                    driveTrain);
  return new SequentialCommandGroup(
          ramseteCommand1,
          new ParallelCommandGroup(
                  new IntakeManipulator(intake, () -> -0.50)
                          .withTimeout(5),
                  new ShootBalls(shooter, () -> -0.75)
                          .withTimeout(7.5)
          ),
          ramseteCommand2
  );
  }

}

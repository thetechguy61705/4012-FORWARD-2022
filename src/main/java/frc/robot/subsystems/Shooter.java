// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.sql.rowset.spi.SyncResolver;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private final WPI_TalonFX shooter_motor_top;
    private final WPI_TalonFX shooter_motor_bottom;
    private final double bottom_RPM = -0.60;

    public Shooter() {
        shooter_motor_top = new WPI_TalonFX(Constants.TOP_SHOOTER_MOTOR);
        shooter_motor_bottom = new WPI_TalonFX(Constants.BOTTOM_SHOOTER_MOTOR);

        // Init motors
        shooter_motor_top.clearStickyFaults();
        shooter_motor_bottom.clearStickyFaults();
        shooter_motor_top.configFactoryDefault();
        shooter_motor_bottom.configFactoryDefault();

        shooter_motor_top.setNeutralMode(NeutralMode.Brake);
        shooter_motor_bottom.setNeutralMode(NeutralMode.Brake);


    }

    @Override
    public void periodic() {
        // Runs once per scheduler run :catsmile:

        SmartDashboard.putNumber("Shooter: Top Motor RPM", getTopRPM());
        SmartDashboard.putNumber("Shooter: Bottom RPM", getBottomRPM());
    }

    public double getTopRPM() {
        return (shooter_motor_top.getSelectedSensorVelocity() / 2048 * 10) * 60.0;
    }
    public double getBottomRPM() {
        return (shooter_motor_bottom.getSelectedSensorVelocity() / 2048 * 10) * 60.0;
    }

    public synchronized void shootBalls(double powerLevel) {
        shooter_motor_top.set(ControlMode.PercentOutput, powerLevel);
        shooter_motor_bottom.set(ControlMode.PercentOutput, bottom_RPM);
    }

    public synchronized void runTop(double powerLevel) {
        shooter_motor_top.set(ControlMode.PercentOutput, powerLevel);
    }

    public synchronized void idleShooter() {
        shooter_motor_bottom.set(ControlMode.PercentOutput, 0);
        shooter_motor_top.set(ControlMode.PercentOutput, 0);

    }

    public synchronized void runBottom() {
        shooter_motor_bottom.set(ControlMode.PercentOutput, bottom_RPM);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}

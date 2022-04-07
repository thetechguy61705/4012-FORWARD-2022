// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final VictorSPX intake_motor;

    public Intake() {
        intake_motor = new VictorSPX(Constants.INTAKE_MOTOR_PORT);

        intake_motor.clearStickyFaults();
        intake_motor.setInverted(InvertType.None);

        intake_motor.configFactoryDefault();
    }
    public synchronized void intakeBalls(double speed) {
        intake_motor.set(ControlMode.PercentOutput, speed);
    }

    public synchronized void stopMotor() {
        intake_motor.set(ControlMode.PercentOutput, 0);
    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}

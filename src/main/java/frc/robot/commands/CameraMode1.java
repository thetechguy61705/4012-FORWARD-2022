package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;


public class CameraMode1 extends CommandBase {
    private final Limelight limelight;

    public CameraMode1(Limelight limelight) {
        this.limelight = limelight;
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.limelight);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        limelight.setCam(Limelight.CamModes.DRIVER);
        limelight.setLed(Limelight.LedModes.OFF);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

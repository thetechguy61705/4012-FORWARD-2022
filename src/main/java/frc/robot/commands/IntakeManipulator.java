package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

import java.util.function.DoubleSupplier;


public class IntakeManipulator extends CommandBase {
    private final Intake intake;
    private final DoubleSupplier speed;

    public IntakeManipulator(Intake intake, DoubleSupplier speed) {
        this.intake = intake;
        this.speed = speed;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.intake);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        SmartDashboard.putString("Intake Enabled", "yes");
        intake.intakeBalls(speed.getAsDouble());
    }
    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Intake Enabled", "no");
        intake.stopMotor();
    }
}

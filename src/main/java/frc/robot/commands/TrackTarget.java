package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

import java.text.DecimalFormat;


public class TrackTarget extends CommandBase {
    private final DriveTrain driveTrain;
    private final Limelight limelight;
    public DecimalFormat dF = new DecimalFormat("0.000");


    double kProt = Limelight.LimelightConstants.kProt;
    double kPdist = Limelight.LimelightConstants.kPdist;

    public TrackTarget(DriveTrain driveTrain, Limelight limelight) {
        this.driveTrain = driveTrain;
        this.limelight = limelight;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain, this.limelight);
    }

    @Override
    public void initialize() {

        // Activates the limelight into the state required to track target.

        limelight.setLed(Limelight.LedModes.ON);
        limelight.setCam(Limelight.CamModes.VISION);

    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Sees Target", limelight.hasTarget());
        double steeringAdjust = kProt * limelight.getTargetX();

        if(limelight.getTargetX() > .8){
            steeringAdjust += Limelight.LimelightConstants.kMin;
        } else if(limelight.getTargetX() < .8){
            steeringAdjust -= Limelight.LimelightConstants.kMin;
        }

        double distanceAdjust = (Limelight.LimelightConstants.desiredArea - limelight.getTargetArea()) * kPdist;


        if(steeringAdjust >= .4) {
            steeringAdjust = .4;
        } else if(steeringAdjust <= -.4) {
            steeringAdjust = -.4;
        }

        if(distanceAdjust > .5) {
            distanceAdjust = .5;
        } else if(distanceAdjust < -.5) {
            distanceAdjust = -.5;
        }

        driveTrain.arcadeDrive(-steeringAdjust, -distanceAdjust);

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.manualDrive(0, 0);
    }
}
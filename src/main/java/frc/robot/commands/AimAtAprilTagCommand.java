package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class AimAtAprilTagCommand extends Command {
    private DriveSubsystem m_driveDriveSubsystem;
    private PIDController m_pidController;
    final String limelightName = "limelight";

    public AimAtAprilTagCommand(DriveSubsystem driveSubsystem) {
        m_driveDriveSubsystem = driveSubsystem;

        m_pidController = new PIDController(0.01, 0, 0);

        m_pidController.setTolerance(3);

        m_pidController.setSetpoint(0);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        m_pidController.reset();
    }

    //runs periodically every .02 seconds   
    @Override
    public void execute() {
        //If we dont see a target, we dont want to do anything
        if (LimelightHelpers.getFiducialID(limelightName) == -1) {
            m_driveDriveSubsystem.setSpeeds(0, 0);
        } else {
            double turningSpeed = m_pidController.calculate(LimelightHelpers.getTX(limelightName));

            m_driveDriveSubsystem.setArcadeSpeed(0, turningSpeed);
        }

        //If we do see a target, we want to used the pidcontroller
    }

    @Override
    public void end(boolean interrupted) {
        m_driveDriveSubsystem.setArcadeSpeed(0, 0);
    }
}

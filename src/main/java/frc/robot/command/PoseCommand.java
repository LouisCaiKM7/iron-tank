package frc.robot.command;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotConstants;
import frc.robot.subsystems.tank.TankSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.RobotConstants.ForwardConstants.ForwardPID.*;

public class PoseCommand extends Command {
    /**
     * Creates a new ForwardCommand.
     */

    PIDController posePIDCtrl = new PIDController(kP.get(), kI.get(), kD.get());
    TankSubsystem mTankSubsystem;
    CommandXboxController Controller;
    Pose2d targetLocation;

    public PoseCommand(Pose2d targetLocation, TankSubsystem mTankSubsystem, CommandXboxController Controller) {
        this.targetLocation = targetLocation;
        this.mTankSubsystem = mTankSubsystem;
        this.Controller = Controller;
        // Use addRequirements() here to declare subsystem dependencies.
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (RobotConstants.TUNING) {
            posePIDCtrl.setPID(kP.get(), kI.get(), kD.get());
        }
        posePIDCtrl.reset();
        posePIDCtrl.setSetpoint(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mTankSubsystem.setArcadeSpeed(
                MetersPerSecond.of(-posePIDCtrl.calculate(
                        targetLocation.getTranslation().minus(mTankSubsystem.getRobotPose().getTranslation()).getNorm())),
                RadiansPerSecond.of(-posePIDCtrl.calculate(
                        (mTankSubsystem.getRobotPose().getRotation().minus((targetLocation.getTranslation().minus(
                                mTankSubsystem.getRobotPose().getTranslation()).getAngle()))).getDegrees()))
        );

        Logger.recordOutput("Tank/targetLocation", targetLocation);
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mTankSubsystem.setArcadeSpeed(
                MetersPerSecond.of(0),
                RadiansPerSecond.of(-posePIDCtrl.calculate(
                        (targetLocation.getRotation().minus(mTankSubsystem.getRobotPose().getRotation()).getDegrees())))
        );
    }

    // Returns true when the command should en// d.
    @Override
    public boolean isFinished() {
        return false;
    }
}

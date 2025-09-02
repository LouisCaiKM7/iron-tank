package frc.robot.command;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotConstants;
import frc.robot.subsystems.tank.TankSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.RobotConstants.PoseConstants.RotationPID.*;
import static frc.robot.RobotConstants.PoseConstants.TranslationPID.*;

public class PoseCommand extends Command {
    /**
     * Creates a new ForwardCommand.
     */

    PIDController translationPIDCtrl = new PIDController(TranKP.get(), TranKI.get(), TranKD.get());
    PIDController rotationPIDCtrl = new PIDController(RotKP.get(), RotKI.get(), RotKD.get());
    TankSubsystem mTankSubsystem;
    CommandXboxController Controller;
    Pose2d targetLocation;
    boolean TranslationAtGoal = false;

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
            translationPIDCtrl.setPID(TranKP.get(), TranKI.get(), TranKD.get());
            rotationPIDCtrl.setPID(RotKP.get(), RotKI.get(), RotKD.get());
        }
        translationPIDCtrl.reset();
        rotationPIDCtrl.reset();

        translationPIDCtrl.setSetpoint(0);
        rotationPIDCtrl.setSetpoint(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (targetLocation.getTranslation().minus(mTankSubsystem.getRobotPose().getTranslation()).getNorm() <= 0.02) {
            TranslationAtGoal = true;
        }

        if (!TranslationAtGoal) {
            mTankSubsystem.setArcadeSpeed(
                    MetersPerSecond.of(-translationPIDCtrl.calculate(
                            targetLocation.getTranslation().minus(mTankSubsystem.getRobotPose().getTranslation()).getNorm())),
                    DegreesPerSecond.of(-rotationPIDCtrl.calculate(
                            (mTankSubsystem.getRobotPose().getRotation().minus(targetLocation.getTranslation().minus(
                                    mTankSubsystem.getRobotPose().getTranslation()).getAngle())).getDegrees()))
            );
        } else {
            mTankSubsystem.setArcadeSpeed(
                    MetersPerSecond.of(0),
                    DegreesPerSecond.of(rotationPIDCtrl.calculate(
                            (targetLocation.getRotation().minus(mTankSubsystem.getRobotPose().getRotation()).getDegrees())))
            );
        }

        Logger.recordOutput("Tank/targetLocation", targetLocation);
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        TranslationAtGoal = false;
    }

    // Returns true when the command should en// d.
    @Override
    public boolean isFinished() {
        return false;
    }
}

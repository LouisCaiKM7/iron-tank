package frc.robot.command;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotConstants;
import frc.robot.subsystems.tank.TankSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.RobotConstants.ForwardConstants.ForwardPID.*;

public class ForwardCommand extends Command {
    /**
     * Creates a new ForwardCommand.
     */

    PIDController forwardPIDCtrl = new PIDController(kP.get(), kI.get(), kD.get());
    TankSubsystem mTankSubsystem;
    Distance targetDistance;
    CommandXboxController Controller;
    Translation2d translationShift;
    Pose2d targetPose = new Pose2d();

    public ForwardCommand(Distance targetDistance, TankSubsystem mTankSubsystem, CommandXboxController Controller) {
        this.targetDistance = targetDistance;
        this.mTankSubsystem = mTankSubsystem;
        this.Controller = Controller;
        translationShift = new Translation2d(targetDistance.magnitude(), 0.0);
        // Use addRequirements() here to declare subsystem dependencies.
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (RobotConstants.TUNING) {
            forwardPIDCtrl.setPID(kP.get(), kI.get(), kD.get());
        }
        forwardPIDCtrl.reset();
        translationShift.rotateBy(mTankSubsystem.getRobotPose().getRotation());
        forwardPIDCtrl.setSetpoint(0);
        targetPose = mTankSubsystem.getRobotPose().plus(new Transform2d(translationShift, new Rotation2d(0)));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mTankSubsystem.setArcadeSpeed(
                MetersPerSecond.of(
                        -forwardPIDCtrl.calculate(
                                targetPose.minus(mTankSubsystem.getRobotPose()).getTranslation().getNorm())),
                RadiansPerSecond.of(0));
        Logger.recordOutput("Tank/Feedback", targetPose.minus(mTankSubsystem.getRobotPose()).getTranslation().getNorm());
        Logger.recordOutput("Tank/targetPose", targetPose);
        Logger.recordOutput("Tank/deltaPose", mTankSubsystem.getRobotPose().minus(targetPose).getTranslation().getNorm());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

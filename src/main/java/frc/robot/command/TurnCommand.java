// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotConstants;
import frc.robot.subsystems.tank.TankSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static frc.robot.RobotConstants.PigeonConstants.PigeonPID.*;
import static frc.robot.RobotContainer.deadBand;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnCommand extends Command {
    /**
     * Creates a new TurnCommand.
     */

    PIDController turningPIDCtrl = new PIDController(kP.get(), kI.get(), kD.get());
    TankSubsystem mTankSubsystem;
    Angle targetAngle;
    CommandXboxController Controller;

    public TurnCommand(Angle targetAngle, TankSubsystem mTankSubsystem, CommandXboxController Controller) {
        this.targetAngle = targetAngle;
        this.mTankSubsystem = mTankSubsystem;
        this.Controller = Controller;
        turningPIDCtrl.reset();
        turningPIDCtrl.setSetpoint(targetAngle.in(Degrees));
        // Use addRequirements() here to declare subsystem dependencies.
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (RobotConstants.TUNING) {
            turningPIDCtrl.setPID(kP.get(), kI.get(), kD.get());
        }

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mTankSubsystem.setArcadeSpeed(
                RobotConstants.TankConstants.MAX_SPEED.times(
                        deadBand(-Controller.getLeftY(), 0.05)).times(0.5),
                DegreesPerSecond.of(
                        turningPIDCtrl.calculate(
                                mTankSubsystem.getRobotPose().getRotation().getDegrees())));

        Logger.recordOutput("Tank/targetAngle", targetAngle);
        Logger.recordOutput("Tank/measuredAngle", mTankSubsystem.getRobotPose().getRotation().getDegrees());

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

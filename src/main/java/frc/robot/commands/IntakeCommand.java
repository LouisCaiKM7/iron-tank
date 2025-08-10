package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

public class IntakeCommand extends Command {

    private IntakeSubsystem intakeSubsystem;

    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setPivotPosition(Degrees.of(RobotConstants.IntakeConstants.INTAKE_POSITION_DEGREES.get()));
        intakeSubsystem.setRollerVoltage(Volts.of(RobotConstants.IntakeConstants.INTAKE_VOLTAGE.get()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setPivotPosition(Degrees.of(RobotConstants.IntakeConstants.ELEVATE_POSITION_DEGREES.get()));
        intakeSubsystem.setRollerVoltage(Volts.of(RobotConstants.IntakeConstants.HOLD_VOLTAGE.get()));
    }
}

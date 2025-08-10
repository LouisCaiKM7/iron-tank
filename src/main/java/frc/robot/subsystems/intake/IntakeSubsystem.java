package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class IntakeSubsystem extends SubsystemBase {

    private RollerIO rollerIO;
    private PivotIO pivotIO;
    private RollerIOInputsAutoLogged rollerIOInputsAutoLogged = new RollerIOInputsAutoLogged();
    private PivotIOInputsAutoLogged pivotIOInputsAutoLogged = new PivotIOInputsAutoLogged();

    public IntakeSubsystem(
            RollerIO rollerIO,
            PivotIO pivotIO
    ) {
        this.pivotIO = pivotIO;
        this.rollerIO = rollerIO;
    }

    public void setPivotPosition(Angle position) {
        pivotIO.setPosition(position);
    }

    public void setRollerVoltage(Voltage voltage) {
        rollerIO.setVoltage(voltage.in(Volts));
    }

    public void setRollerVelocity(AngularVelocity velocity) {
        rollerIO.setVelocity(velocity.in(RotationsPerSecond));
    }

    @Override
    public void periodic() {
        rollerIO.updateInputs(rollerIOInputsAutoLogged);
        pivotIO.updateInputs(pivotIOInputsAutoLogged);
        Logger.processInputs("Intake/Roller", rollerIOInputsAutoLogged);
        Logger.processInputs("Intake/Pivot", pivotIOInputsAutoLogged);

        if (RobotConstants.TUNING) {
            pivotIO.updateConfigs(
                    RobotConstants.IntakeConstants.IntakePivotPID.kP.get(),
                    RobotConstants.IntakeConstants.IntakePivotPID.kI.get(),
                    RobotConstants.IntakeConstants.IntakePivotPID.kD.get(),
                    RobotConstants.IntakeConstants.IntakePivotPID.kA.get(),
                    RobotConstants.IntakeConstants.IntakePivotPID.kV.get(),
                    RobotConstants.IntakeConstants.IntakePivotPID.kS.get(),
                    RobotConstants.IntakeConstants.IntakePivotPID.kG.get());
            rollerIO.updateConfigs(
                    RobotConstants.IntakeConstants.IntakeRollerPID.kP.get(),
                    RobotConstants.IntakeConstants.IntakeRollerPID.kI.get(),
                    RobotConstants.IntakeConstants.IntakeRollerPID.kD.get(),
                    RobotConstants.IntakeConstants.IntakeRollerPID.kA.get(),
                    RobotConstants.IntakeConstants.IntakeRollerPID.kV.get(),
                    RobotConstants.IntakeConstants.IntakeRollerPID.kS.get());
        }

    }
}

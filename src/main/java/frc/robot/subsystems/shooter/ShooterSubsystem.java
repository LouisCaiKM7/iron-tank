package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class ShooterSubsystem extends SubsystemBase {

    private RollerIO io;
    private RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

    public ShooterSubsystem(RollerIO io) {
        this.io = io;
        io.updateConfigs(
                RobotConstants.ShooterConstants.ShooterPID.kP.get(),
                RobotConstants.ShooterConstants.ShooterPID.kI.get(),
                RobotConstants.ShooterConstants.ShooterPID.kD.get(),
                RobotConstants.ShooterConstants.ShooterPID.kA.get(),
                RobotConstants.ShooterConstants.ShooterPID.kV.get(),
                RobotConstants.ShooterConstants.ShooterPID.kS.get()
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        if (RobotConstants.TUNING) {
            io.updateConfigs(
                    RobotConstants.ShooterConstants.ShooterPID.kP.get(),
                    RobotConstants.ShooterConstants.ShooterPID.kI.get(),
                    RobotConstants.ShooterConstants.ShooterPID.kD.get(),
                    RobotConstants.ShooterConstants.ShooterPID.kA.get(),
                    RobotConstants.ShooterConstants.ShooterPID.kV.get(),
                    RobotConstants.ShooterConstants.ShooterPID.kS.get()
            );
        }
    }

    public void setVoltage(Voltage voltage) {
        io.setVoltage(voltage.in(Volts));
    }

    public void setVelocity(AngularVelocity velocity) {
        io.setVelocity(velocity.in(RotationsPerSecond));
    }
}
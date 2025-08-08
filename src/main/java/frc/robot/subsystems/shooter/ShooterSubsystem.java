package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerSubsystem;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class ShooterSubsystem extends RollerSubsystem {

    private RollerIO io;

    public ShooterSubsystem(RollerIO io) {
        super(io, "Shooter");
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

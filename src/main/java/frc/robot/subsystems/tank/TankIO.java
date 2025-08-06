package frc.robot.subsystems.tank;

import frc.robot.RobotConstants;
import org.littletonrobotics.junction.AutoLog;

public interface TankIO {
    void setRPS(double leftRPS, double rightRPS);

    void updateInputs(TankIOInputs inputs);

    @AutoLog
    class TankIOInputs {
        public double leftMotorVelocityRotPerSec = 0.0;
        public double leftMotorAppliedVolts = 0.0;
        public double leftMotorStatorCurrentAmps = 0.0;
        public double leftMotorSupplyCurrentAmps = 0.0;
        public double leftMotorTempCelsius = 0.0;

        public double rightMotorVelocityRotPerSec = 0.0;
        public double rightMotorAppliedVolts = 0.0;
        public double rightMotorStatorCurrentAmps = 0.0;
        public double rightMotorSupplyCurrentAmps = 0.0;
        public double rightMotorTempCelsius = 0.0;

        public double tankKP = RobotConstants.TankConstants.TankPID.kP.get();
        public double tankKI = RobotConstants.TankConstants.TankPID.kI.get();
        public double tankKD = RobotConstants.TankConstants.TankPID.kD.get();
    }
}
package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    void setPosition(Angle position);

    void updateConfigs(double kp, double ki, double kd, double ka, double kv, double ks, double kg);

    void updateInputs(PivotIOInputs inputs);

    @AutoLog
    class PivotIOInputs {
        public double velocityRotPerSec = 0.0;
        public double currentPositionRot = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }
}


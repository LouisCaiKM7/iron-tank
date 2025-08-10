package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Rotations;

public class PivotIOSim implements PivotIO {

    private Angle position = Rotations.of(0);

    public PivotIOSim() {

    }

    @Override
    public void setPosition(Angle position) {
        this.position = position;
        return;
    }

    @Override
    public void updateConfigs(double kp, double ki, double kd, double ka, double kv, double ks, double kg) {
        return;
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.appliedVolts = 0.0;
        inputs.statorCurrentAmps = 0.0;
        inputs.supplyCurrentAmps = 0.0;
        inputs.tempCelsius = 0.0;
        inputs.velocityRotPerSec = 0.0;
        inputs.currentPositionRot = position.in(Rotations);
    }
}

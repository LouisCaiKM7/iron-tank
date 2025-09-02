package frc.robot.subsystems.tank;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public class TankIOReal implements TankIO {
    TalonFX motorRight = new TalonFX(1, "rio");
    private final StatusSignal<AngularVelocity> rightMotorVelocityRotPerSec = motorRight.getVelocity();
    private final StatusSignal<Voltage> rightMotorAppliedVolts = motorRight.getSupplyVoltage();
    private final StatusSignal<Current> rightMotorStatorCurrentAmps = motorRight.getStatorCurrent();
    private final StatusSignal<Current> rightMotorSupplyCurrentAmps = motorRight.getSupplyCurrent();
    private final StatusSignal<Temperature> rightMotorTempCelsius = motorRight.getDeviceTemp();
    private final StatusSignal<Angle> rightPosition = motorRight.getPosition();
    TalonFX motorLeft = new TalonFX(0, "rio");
    private final StatusSignal<AngularVelocity> leftMotorVelocityRotPerSec = motorLeft.getVelocity();
    private final StatusSignal<Voltage> leftMotorAppliedVolts = motorLeft.getSupplyVoltage();
    private final StatusSignal<Current> leftMotorStatorCurrentAmps = motorLeft.getStatorCurrent();
    private final StatusSignal<Current> leftMotorSupplyCurrentAmps = motorLeft.getSupplyCurrent();
    private final StatusSignal<Temperature> leftMotorTempCelsius = motorLeft.getDeviceTemp();
    private final StatusSignal<Angle> leftPosition = motorLeft.getPosition();
    TalonFXConfigurator motorRightConfigurator = motorRight.getConfigurator();
    TalonFXConfigurator motorLeftConfigurator = motorLeft.getConfigurator();

    public TankIOReal() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        motorLeftConfigurator.apply(motorOutputConfigs);
        motorRightConfigurator.apply(motorOutputConfigs);
    }

    public void setRPS(AngularVelocity leftRPS, AngularVelocity rightRPS) {
        motorLeft.setControl(new VelocityVoltage(-leftRPS.in(RadiansPerSecond)));
        motorRight.setControl(new VelocityVoltage(rightRPS.in(RadiansPerSecond)));

        Logger.recordOutput("DriveSubsystem/TargetLeftRPS", leftRPS.in(RadiansPerSecond));
        Logger.recordOutput("DriveSubsystem/TargetRightRPS", rightRPS.in(RadiansPerSecond));

    }

    @Override
    public void updateInputs(TankIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                leftMotorAppliedVolts,
                leftMotorStatorCurrentAmps,
                leftMotorSupplyCurrentAmps,
                leftMotorTempCelsius,
                leftMotorVelocityRotPerSec,
                leftPosition,
                rightMotorAppliedVolts,
                rightMotorStatorCurrentAmps,
                rightMotorSupplyCurrentAmps,
                rightMotorTempCelsius,
                rightMotorVelocityRotPerSec,
                rightPosition
        );
        inputs.leftMotorAppliedVolts = leftMotorAppliedVolts.getValueAsDouble();
        inputs.leftMotorStatorCurrentAmps = leftMotorStatorCurrentAmps.getValueAsDouble();
        inputs.leftMotorSupplyCurrentAmps = leftMotorSupplyCurrentAmps.getValueAsDouble();
        inputs.leftMotorTempCelsius = leftMotorTempCelsius.getValueAsDouble();
        inputs.leftMotorVelocityRotPerSec = leftMotorVelocityRotPerSec.getValueAsDouble();
        inputs.leftPosition = leftPosition.getValueAsDouble();
        inputs.rightMotorAppliedVolts = rightMotorAppliedVolts.getValueAsDouble();
        inputs.rightMotorStatorCurrentAmps = rightMotorStatorCurrentAmps.getValueAsDouble();
        inputs.rightMotorSupplyCurrentAmps = rightMotorSupplyCurrentAmps.getValueAsDouble();
        inputs.rightMotorTempCelsius = rightMotorTempCelsius.getValueAsDouble();
        inputs.rightMotorVelocityRotPerSec = rightMotorVelocityRotPerSec.getValueAsDouble();
        inputs.rightPosition = rightPosition.getValueAsDouble();
        if (RobotConstants.TUNING) {
            motorLeft.getConfigurator().apply(new Slot0Configs()
                    .withKP(RobotConstants.TankConstants.TankPID.kP.get())
                    .withKI(RobotConstants.TankConstants.TankPID.kI.get())
                    .withKD(RobotConstants.TankConstants.TankPID.kD.get()));
            motorRight.getConfigurator().apply(new Slot0Configs()
                    .withKP(RobotConstants.TankConstants.TankPID.kP.get())
                    .withKI(RobotConstants.TankConstants.TankPID.kI.get())
                    .withKD(RobotConstants.TankConstants.TankPID.kD.get()));
        }
    }
}
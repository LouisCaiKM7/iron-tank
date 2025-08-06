// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tank;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotConstants.TankConstants.*;

public class TankSubsystem extends SubsystemBase {

    private final TankIO io;
    private final TankIOInputsAutoLogged inputs = new TankIOInputsAutoLogged();

    public TankSubsystem(TankIO io) {
        this.io = io;
    }

    /**
     * @param forwardSpeed: m/s
     * @param turningSpeed: rad/s
     */
    public void setArcadeSpeed(double forwardSpeed, double turningSpeed) {
        double forwardRPS = forwardSpeed / 2 / Math.PI / WHEEL_RADIUS * GEAR_RATIO;
        double turningRPS = turningSpeed * WHEEL_TRACK / 2 * GEAR_RATIO;

        Logger.recordOutput("TankSubsystem/targetForwardSpeed", forwardSpeed);
        Logger.recordOutput("TankSubsystem/targetTurningSpeed", turningSpeed);

        io.setRPS(forwardRPS + turningRPS, forwardRPS - turningRPS);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("TANK",inputs);
    }
}

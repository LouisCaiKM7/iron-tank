// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tank;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotConstants.TankConstants.*;


public class TankSubsystem extends SubsystemBase {

    private final TankIO io;
    private final TankIOInputsAutoLogged inputs = new TankIOInputsAutoLogged();
    Pose2d robotPose = new Pose2d();
    // 添加基于 RPS 的位置计算变量
    private double lastLeftRPS = 0.0;
    private double lastRightRPS = 0.0;
    private double currentX = 0.0;
    private double currentY = 0.0;
    private double currentAngle = 0.0;
    private double lastTime = 0.0;
    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(WHEEL_TRACK);

    public TankSubsystem(TankIO io) {
        this.io = io;
    }

    /**
     * @param forwardSpeed: m/s
     * @param turningSpeed: rad/s
     */
    public void setArcadeSpeed(LinearVelocity forwardSpeed, AngularVelocity turningSpeed) {
        DifferentialDriveWheelSpeeds wheelSpeeds =
                kinematics.toWheelSpeeds(
                        new ChassisSpeeds(forwardSpeed, MetersPerSecond.of(0), turningSpeed));

        SmartDashboard.putNumber("TankSubsystem/forwardSpeed", forwardSpeed.in(MetersPerSecond));
        SmartDashboard.putNumber("TankSubsystem/turningSpeed", turningSpeed.in(RadiansPerSecond));

        io.setRPS(chassisSpeedToMotorRPS(wheelSpeeds.leftMetersPerSecond), chassisSpeedToMotorRPS(wheelSpeeds.rightMetersPerSecond));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Tank", inputs);
        updatePoseFromRPS();
        
        Logger.recordOutput("Tank/RobotPose", robotPose);
        System.out.println(robotPose);
    }

    private AngularVelocity chassisSpeedToMotorRPS(double chassisSpeedMetersPerSecond) {
        return RotationsPerSecond.of(chassisSpeedMetersPerSecond / (WHEEL_RADIUS.in(Meters) * 2 * Math.PI)).times(GEAR_RATIO);
    }

    private void updatePoseFromRPS() {
        double currentTime = System.currentTimeMillis() / 1000.0; // 转换为秒
        double deltaTime = currentTime - lastTime;

        if (lastTime == 0.0) {
            lastTime = currentTime;
            lastLeftRPS = inputs.leftMotorVelocityRotPerSec;
            lastRightRPS = inputs.rightMotorVelocityRotPerSec;
            return;
        }

        // 计算左右轮的平均 RPS
        double avgLeftRPS = (inputs.leftMotorVelocityRotPerSec + lastLeftRPS) / 2.0;
        double avgRightRPS = (inputs.rightMotorVelocityRotPerSec + lastRightRPS) / 2.0;

        // 将 RPS 转换为线速度 (m/s)
        double leftWheelSpeed = avgLeftRPS / GEAR_RATIO * (WHEEL_RADIUS.in(Meters) * 2 * Math.PI);
        double rightWheelSpeed = avgRightRPS / GEAR_RATIO * (WHEEL_RADIUS.in(Meters) * 2 * Math.PI);

        // 计算前进速度和角速度
        double forwardSpeed = (leftWheelSpeed + rightWheelSpeed) / 2.0;
        double angularSpeed = (rightWheelSpeed - leftWheelSpeed) / WHEEL_TRACK.in(Meters);

        // 积分计算位置变化
        double deltaDistance = forwardSpeed * deltaTime;
        double deltaAngle = angularSpeed * deltaTime;

        // 更新角度
        currentAngle += deltaAngle;

        // 计算位置变化（考虑当前角度）
        double deltaX = deltaDistance * Math.cos(currentAngle);
        double deltaY = deltaDistance * Math.sin(currentAngle);

        // 更新位置
        currentX += deltaX;
        currentY += deltaY;

        // 更新 robotPose
        robotPose = new Pose2d(currentX, currentY, new Rotation2d(currentAngle));

        SmartDashboard.putNumber("TankSubsystem/leftRPS", inputs.leftMotorVelocityRotPerSec);
        SmartDashboard.putNumber("TankSubsystem/rightRPS", inputs.rightMotorVelocityRotPerSec);

        // 保存当前值作为下次计算的 last 值
        lastLeftRPS = inputs.leftMotorVelocityRotPerSec;
        lastRightRPS = inputs.rightMotorVelocityRotPerSec;
        lastTime = currentTime;
    }
}
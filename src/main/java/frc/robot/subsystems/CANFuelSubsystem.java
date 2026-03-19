// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Launch;

import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax LeftIntakeLauncher;
  private final SparkMax RightIntakeLauncher;
  private final SparkMax Indexer;
  private final RelativeEncoder LauncherEncoder;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    LeftIntakeLauncher = new SparkMax(LEFT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    RightIntakeLauncher = new SparkMax(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    Indexer = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless);
    LauncherEncoder = LeftIntakeLauncher.getEncoder();

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(INDEXER_MOTOR_CURRENT_LIMIT);
    feederConfig.inverted(true);
    Indexer.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();

    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    launcherConfig.voltageCompensation(12);
    launcherConfig.idleMode(IdleMode.kCoast);
    RightIntakeLauncher.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    launcherConfig.inverted(true);
    LeftIntakeLauncher.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INDEXER_INTAKING_PERCENT);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKE_INTAKING_PERCENT);
    SmartDashboard.putNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT);
    SmartDashboard.putNumber("Launching launcher RPM", 2150);
    SmartDashboard.putNumber("Shooter KF", 0.000166);
    SmartDashboard.putNumber("Shooter KP", 0.0007);
    //SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  // A method to set the voltage of the intake roller
  public void setIntakeLauncherRoller(double power) {
    LeftIntakeLauncher.set(power);
    RightIntakeLauncher.set(power); // positive for shooting
  }

  // A method to set the voltage of the intake roller
  public void setFeederRoller(double power) {
    Indexer.set(power); // positive for shooting
  }

  // A method to stop the rollers
  public void stop() {
    Indexer.set(0);
    LeftIntakeLauncher.set(0);
    RightIntakeLauncher.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPM", LauncherEncoder.getVelocity());
  }

  public void spintAtTunableRpm() {
    double goalRpm = SmartDashboard.getNumber("Launching launcher RPM", 5000);
    double kf = SmartDashboard.getNumber("Shooter KF", 0);
    double kp = SmartDashboard.getNumber("Shooter KP", 0);

    double error = goalRpm - LauncherEncoder.getVelocity();
    double output = kf * goalRpm + kp * error;

    setIntakeLauncherRoller(output);
  }
  

  public Command createTestShooterCommand() {
      return runEnd(() -> {
        double percent = SmartDashboard.getNumber("Launching launcher roller value", INTAKE_INTAKING_PERCENT);
        setIntakeLauncherRoller(percent);
      }, this::stop).withName("Test Shooter");
  }

  public Command createMoveLeftIntakeLauncherCommand() {
    return runEnd(() -> {
      LeftIntakeLauncher.set(0.5);

    }, this::stop).withName("Create L Shooter");
  }

  public Command createMoveRightIntakeLauncherCommand() {
    return runEnd(() -> {
      RightIntakeLauncher.set(0.5);

    }, this::stop).withName("Create Move R Shooter");
  }
  

  public Command createMoveIndexerCommand() {
    return runEnd(() -> {
      Indexer.set(0.5);

    }, this::stop).withName("Create Move Indexer");
  }

  public Command createSpinAtTunableRpm() {
    return runEnd(this::spintAtTunableRpm, this::stop);
  }
  
    // LeftIntakeLauncher = new SparkMax(LEFT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    // RightIntakeLauncher = new SparkMax(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    // Indexer = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless);
}

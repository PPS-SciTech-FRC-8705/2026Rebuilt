// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDrive extends Command {
  /** Creates a new Drive. */
  private final CommandSwerveDrivetrain driveSubsystem;
  private final double xSpeed, ySpeed, zRotation;
  
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(RobotContainer.MaxSpeed * 0.1).withRotationalDeadband(RobotContainer.MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest idle = new SwerveRequest.Idle();

  public AutoDrive(CommandSwerveDrivetrain driveSystem, double xSpeed, double ySpeed, double zRotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zRotation = zRotation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Setting the values here instead of in initialize feeds the watchdog on the
  // arcade drive object
  @Override
  public void execute() {
    driveSubsystem.setControl(
        drive.withVelocityX(xSpeed * RobotContainer.MaxSpeed)
            .withVelocityY(ySpeed * RobotContainer.MaxSpeed)
            .withRotationalRate(zRotation * RobotContainer.MaxAngularRate)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setControl(idle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
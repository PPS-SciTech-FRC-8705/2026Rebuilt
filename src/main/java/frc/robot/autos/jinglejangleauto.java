// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.io.IOException;
import java.nio.file.Path;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.LaunchSequence;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class jinglejangleauto extends SequentialCommandGroup {
  /** Creates a new ExampleAuto. */
  public jinglejangleauto(CommandSwerveDrivetrain driveSubsystem, CANFuelSubsystem ballSubsystem, ClimberSubsystem climbersubsystem) {
    String pathName = "jingle jangle auto";
    Pose2d startingPose = driveSubsystem.getPathStartingPose(pathName);
  
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        AutoBuilder.resetOdom(startingPose),
        driveSubsystem.createFollowPathplanerPathCommand(pathName),
        new LaunchSequence(ballSubsystem).withTimeout(7),
        driveSubsystem.createFollowPathplanerPathCommand("JingleJangleToOutpost"),
        new WaitCommand(3),
        driveSubsystem.createFollowPathplanerPathCommand("JingleJangleTower"),
        new LaunchSequence(ballSubsystem).withTimeout(7)


        
    );
  }
}
package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;



// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

public class MoveToPosition extends CommandBase {
    /** Moves the robot to the desired position */

    // Main defines;

    Swerve s_Swerve;
    Vision vision;

    public MoveToPosition(Swerve s_Swerve, Vision vision) {
        this.s_Swerve = s_Swerve;
        this.vision = vision;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(s_Swerve, vision);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        // System.out.println("Waiting for trajectory");
        
        // var optionalTarget = vision.getTarget();
        // if (optionalTarget.isEmpty()) {
        //     return;
        // }
        // PhotonTrackedTarget target = optionalTarget.get();
        
        // var toTargetTrajectory = vision.getTrajectory(target);
        // if (toTargetTrajectory.isEmpty()) {
        //     System.err.println("Unable to generate trajectory");
        //     return;
        // }

        // System.out.println("Has trajectory");
        // Trajectory teleopTrajectory = toTargetTrajectory.get();

        
        System.out.println("Running Teleop Trajectory.");

        Trajectory teleopTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0 ,new Rotation2d(0, 0)),
            List.of(new Translation2d(0, 0)),
            new Pose2d(0, 0, new Rotation2d(Math.PI)),
            Constants.Trajectory.CONFIG
        );

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                teleopTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        s_Swerve.resetOdometry(teleopTrajectory.getInitialPose());
        swerveControllerCommand.schedule();
    }
}

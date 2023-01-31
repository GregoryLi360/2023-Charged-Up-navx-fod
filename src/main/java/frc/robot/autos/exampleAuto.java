package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class exampleAuto extends SequentialCommandGroup {

    private Trajectory driveToTarget(TrajectoryConfig config, Vision vision) {
        var res = vision.get();
        if (res.isEmpty()) {
            System.out.println("Result is empty");
            return null;
        }

        Transform3d transform = res.get();
        Translation2d end = transform.getTranslation().toTranslation2d().minus(new Translation2d(0.5, 0)).times(coefficient);

        /* Pose2d start, List<Translation2D> pathPoints, Pose2d end, config */
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(end.div(2)),
            new Pose2d(end, transform.getRotation().toRotation2d().times(-1)),
            config
        );

        
    }

    public static final double coefficient = 1.097;
    public exampleAuto(Swerve s_Swerve, Vision vision) {
        TrajectoryConfig config = new TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics
        );

        Trajectory driveToTarget = driveToTarget(config, vision);

        var thetaController = new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                driveToTarget,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve
            );


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(driveToTarget.getInitialPose())),
            swerveControllerCommand
        );
    }
}
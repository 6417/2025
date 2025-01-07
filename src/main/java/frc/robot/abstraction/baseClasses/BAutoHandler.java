package frc.robot.abstraction.baseClasses;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.fridowpi.module.Module;

/**
 * BAutoHandler
 */
public abstract class BAutoHandler extends Module {

	abstract public Command getAutoCommand(Trajectory tra, Rotation2d endRot);

	abstract public ChassisSpeeds getVelocitiesAtTimepoint(Trajectory tra, double t, Rotation2d endRot);

	abstract public Command getPoseCommand(Pose2d startPose, Rotation2d endRot);
}

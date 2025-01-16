package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.swerve.SwerveDrive;

public class ChaseTagCommand extends Command {

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private static final int TAG_TO_CHASE = 2;
    private static final Transform3d TAG_TO_GOAL = new Transform3d(
            new Translation3d(1.5, 0, 0), // x, y, z
            new Rotation3d(0, 0, Math.PI)); // roll, pitch, yaw
    private final SwerveDrive swerveDriveSubsystem;

    private final ProfiledPIDController xController = new ProfiledPIDController(0, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(0, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(0, 0, 0, OMEGA_CONSTRAINTS);
    
    private double lastTarget;

    public ChaseTagCommand(SwerveDrive swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        lastTarget = -1;
        Pose2d robotPose = swerveDriveSubsystem.getPose();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    @Override
    public void execute() {
        Pose2d robotPose2d = swerveDriveSubsystem.getPose();
        Pose3d robotPose = new Pose3d(
                robotPose2d.getX(),
                robotPose2d.getY(),
                0.0,
                new Rotation3d(0, 0, robotPose2d.getRotation().getRadians())); // x, y, z, roll, pitch, yaw

        if (LimelightHelpers.getTV(Constants.Limelight.limelightID)) {
            // Find the tag we want to chase
            double target = LimelightHelpers.getFiducialID(Constants.Limelight.limelightID);

            // This is new target data, so recalculate the goal
            lastTarget = target;

            // Transform the tag's pose to set our goal
            var goalPose = LimelightHelpers.getTargetPose3d_CameraSpace(Constants.Limelight.limelightID);

            // Drive
            xController.setGoal(goalPose.getX());
            yController.setGoal(goalPose.getY());
            omegaController.setGoal(goalPose.getRotation().getAngle());

        }

        if (lastTarget == -1) {
            // No target has been visible, so just stop
            swerveDriveSubsystem.stopMotors();
        } else {
            var xSpeed = xController.calculate(robotPose.getX());
            if (xController.atGoal()) {
                xSpeed = 0;
            }
            var ySpeed = yController.calculate(robotPose.getY());
            if (yController.atGoal()) {
                ySpeed = 0;
            }
            var omegaSpeed = omegaController.calculate(robotPose.getRotation().getAngle());
            if (omegaController.atGoal()) {
                omegaSpeed = 0;
            }

            swerveDriveSubsystem.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed,
                    robotPose2d.getRotation()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stopMotors();
    }

}

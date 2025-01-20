package frc.robot.commands;

import java.util.concurrent.CountDownLatch;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
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

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(2.5, 1.5);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(2.5, 1.5);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 6);

    private final int tagToChase;
    private static final Transform3d TAG_TO_GOAL = new Transform3d(
            new Translation3d(-1, 0, 0), // x, y, z
            new Rotation3d(0, 0, Math.PI)); // roll, pitch, yaw
    private final double[] offset;
    private final SwerveDrive swerveDriveSubsystem;

    private final ProfiledPIDController xController = new ProfiledPIDController(1.5, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(1.5, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(1, 0, 0, OMEGA_CONSTRAINTS);

    private double lastTarget;

    public ChaseTagCommand(SwerveDrive swerveDriveSubsystem, int tagToChase, double[] offset) {
        this.offset = offset;
        this.tagToChase = tagToChase;
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
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    @Override
    public void execute() {
        Pose2d robotPose2d = swerveDriveSubsystem.getPose();

        if (LimelightHelpers.getFiducialID(Constants.Limelight.limelightID) == tagToChase) {
            // Find the tag we want to chase
            double target = LimelightHelpers.getFiducialID(Constants.Limelight.limelightID);

            // This is new target data, so recalculate the goal
            lastTarget = target;

            // Transform the tag's pose to set our goal
            double xDistance = LimelightHelpers.getTargetPose3d_RobotSpace(Constants.Limelight.limelightID).getZ() - offset[0];
            double yDistance = -LimelightHelpers.getTargetPose3d_RobotSpace(Constants.Limelight.limelightID).getX() - offset[1];
            double rRotation = - LimelightHelpers.getTargetPose3d_RobotSpace(Constants.Limelight.limelightID).getRotation().getY() - offset[2];
            
            new Rotation2d();
            Pose2d goalPose = robotPose2d.plus(new Transform2d(xDistance, yDistance, Rotation2d.fromRadians(rRotation)));
            /*var goalPose = robotPose.plus(new Transform3d(new Pose3d(0.0, 0.0, 0.0, new Rotation3d()),
                    LimelightHelpers.getTargetPose3d_RobotSpace(Constants.Limelight.limelightID)));*/

            //goalPose = goalPose.plus(TAG_TO_GOAL);

            // returns the target pose in field space, calculatet by adding the target pose
            // in robot space to the robot pose in field space}

            // Drive
            xController.setGoal(goalPose.getX());
            yController.setGoal(goalPose.getY());
            omegaController.setGoal(goalPose.getRotation().getRadians());

            var xSpeed = xController.calculate(robotPose2d.getX());
            if (xController.atGoal()) {
                xSpeed = 0;
            }
            var ySpeed = yController.calculate(robotPose2d.getY());
            if (yController.atGoal()) {
                ySpeed = 0;
            }
            double omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
            if (omegaController.atGoal()) {
                omegaSpeed = 0;
            }
            var chas = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed,
            robotPose2d.getRotation());
            
            swerveDriveSubsystem.setChassisSpeeds(chas);

        }else {
            swerveDriveSubsystem.stopMotors();
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stopMotors();
    }

}

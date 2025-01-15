package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class ChaseTagCommand extends CommandBase {

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private static final int TAG_TO_CHASE = 2;
    private static final Transform3d TAG_TO_GOAL = new Transform3d(
            new Translation3d(1.5, 0, 0), // x, y, z
            new Rotation3d(0, 0, Math.PI)); // roll, pitch, yaw
    private final Limelight limelight;
    private finalk DriveTrainSubsystem driveTrainSubsystem;
    private final Supplier<Pose2d> poseProvider;

    private final ProfiledPIDController xController = new ProfiledPIDController(0, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(0, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(0, 0, 0, X_CONSTRAINTS);

    private LimelightTarget_Fiducial lastTarget;

    public ChaseTagCommand(Limelight limelight, DriveTrainSubsystem driveTrainSubsystem,
            Supplier<Pose2d> poseProvider) {
        this.limelight = limelight;
        this.driveTrainSubsystem = driveTrainSubsystem;
        this.poseProvider = poseProvider;

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        lastTarget = null;
        Pose2d robotPose = poseProvider.get();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    @Override
    public void execute() {
        Pose2d robotPose2d = LimelightHelpers.getBotPose2d("");
        Pose3d robotPose = new Pose3d(
                robotPose2d.getX(),
                robotPose2d.getY(),
                0.0,
                new Rotation3d(0, 0, robotPose2d.getRotation().getRadians())); // x, y, z, roll, pitch, yaw

        if (LimelightHelpers.getTV("")) {
            // Find the tag we want to chase
            var target = LimelightHelpers.getFiducialID("");
            if (targetOpt.isPrsent()) {
                var target = targetOpt.get();
                // This is new target data, so recalculate the goal
                lastTarget = target;

                // Transform the robot's pose to find the camera's pose
                var cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

                // Transform the camera's pose to find the goal's pose
                var camToTarget = target.getBestCameraToTarget();
                var targetPose = cameraPose.transformBy(camToTarget);

                // Transform the tag's pose to set our goal
                var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

                // Drive
                xController.setGoal(goalPose.getX());
                yController.setGoal(goalPose.getY());
                omegaController.setGoal(goalPose.getRotation().getRadians());
            }
        }

        if (lastTarget == null) {
            // No target has been visible, so just stop
            driveTrainSubsystem.stop();
        } else {
            var xSpeed = xController.calculate(robotPose.getX());
            if (xController.atGoal()) {
                xSpeed = 0;
            }
            var ySpeed = yController.calculate(robotPose.getY());
            if (yController.atGoal()) {
                ySpeed = 0;
            }
            var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
            if (omegaController.atGoal()) {
                omegaSpeed = 0;
            }

            driveTrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed,
                    robotPose2d.getRotation.getRadians()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveTrainSubsystem.stop();
    }

}

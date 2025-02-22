package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.swerve.SwerveDrive;

public class ChaseTagCommand extends Command {

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(3.5, 6);

    private final double[] offset;
    private final SwerveDrive swerveDriveSubsystem;

    private final ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(2, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2.5, 0, 0, OMEGA_CONSTRAINTS);

    private double lastTarget;

    public ChaseTagCommand(SwerveDrive swerveDriveSubsystem, double[] offset) {
        this.offset = offset;
        this.swerveDriveSubsystem = swerveDriveSubsystem;

        xController.setTolerance(0.01);
        yController.setTolerance(0.01);
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

        if (LimelightHelpers.getFiducialID(Constants.Limelight.limelightID) != -1) {

            Pose3d robotPoseInTargetSpace = LimelightHelpers
                    .toPose3D(LimelightHelpers.getBotPose_TargetSpace(Constants.Limelight.limelightID));
            // Find the tag we want to chase
            double target = LimelightHelpers.getFiducialID(Constants.Limelight.limelightID);

            // This is new target data, so recalculate the goal
            lastTarget = target;

            double xDistance = -robotPoseInTargetSpace.getZ() - offset[0];
            double yDistance = robotPoseInTargetSpace.getX() - offset[1];
            double rRotation = robotPoseInTargetSpace.getRotation().getY() - offset[2];
            // new Rotation2d();
            Pose2d goalPose = robotPose2d // robot pose in fieldspace
                    .plus(new Transform2d(xDistance, yDistance, Rotation2d.fromRadians(rRotation)));

            SmartDashboard.putNumber("ChaseTag GoalPose X", goalPose.getX());
            SmartDashboard.putNumber("ChaseTag GoalPose Y", goalPose.getY());

            // returns the target pose in field space, calculatet by adding the target pose
            // in robot space to the robot pose in field space}

            // Drive
            xController.setGoal(goalPose.getX());
            yController.setGoal(goalPose.getY());
            omegaController.setGoal(goalPose.getRotation().getRadians());


            //System.out.println("OmegaGoal" + omegaController.getGoal().toString());

        }

        if (lastTarget != -1) {
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
        }
    }

    @Override
    public boolean isFinished() {
        return (xController.atGoal() && yController.atGoal() && omegaController.atGoal());
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stopMotors();
    }
}

package frc.robot.swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.utils.AccelerationLimiter;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class SwerveDrive extends SubsystemBase {
    public SwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    public SwerveDrivePoseEstimator poseEstimator;

    public LimelightHelpers.PoseEstimate mt2;

    // I made a mistake by a factor of 10 when I considered the force, the accelration should 
    // be roughly 10 m / s^2 and not 75.
    private AccelerationLimiter accelLimiter = new AccelerationLimiter(9, 0.267);

    ChassisSpeeds lastSpeeds = new ChassisSpeeds();

    public static final int LOC_FL = 0;
    public static final int LOC_FR = 1;
    public static final int LOC_RL = 2;
    public static final int LOC_RR = 3;

    Thread odometryThread;

    public SwerveDrive(ModuleConfig[] configs) {
        String[] moduleNames = new String[4];
        moduleNames[LOC_FR] = "Front Right";
        moduleNames[LOC_FL] = "Front Left";
        moduleNames[LOC_RR] = "Rear Right";
        moduleNames[LOC_RL] = "Rear Left";

        modules = new SwerveModule[4];
        for (int i = 0; i < 4; i++) {
            configs[i].name = moduleNames[i];
            modules[i] = new SwerveModule(configs[i]);
            Shuffleboard.getTab("Drive").add("SwerveModule " + moduleNames[i], modules[i]);
        }
        Shuffleboard.getTab("Drive").add("SwerveDrive", this);

        kinematics = new SwerveDriveKinematics(
                configs[0].moduleOffset,
                configs[1].moduleOffset,
                configs[2].moduleOffset,
                configs[3].moduleOffset);

        poseEstimator = new SwerveDrivePoseEstimator(kinematics,
                RobotContainer.getGyroRotation2d(),
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.02, 0.02, 0.01),
                VecBuilder.fill(0.1, 0.1, 0.01));

        setDefaultCommand(new DriveCommand(this));
    }

    long lastSetpointTime = -1;
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, 0.02); // remove the skew

        long timeNow = System.currentTimeMillis();
        if (lastSetpointTime > 0) {
            speeds = accelLimiter.constrain(lastSpeeds, speeds, ((double) (timeNow - lastSetpointTime)) / (double) 1000.0);
        }

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(moduleStates[i]);
        }
        lastSpeeds = speeds;
        lastSetpointTime = timeNow;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(modules[0].getState(), modules[1].getState(),
                modules[2].getState(), modules[3].getState());
    }

    public void periodic() {
        updateOdometry();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    // @SuppressWarnings("removal")
    public  void updateOdometry() {
        poseEstimator.update(
                RobotContainer.getGyroRotation2d(),
                new SwerveModulePosition[] {
                        modules[LOC_FL].getPosition(),
                        modules[LOC_FR].getPosition(),
                        modules[LOC_RL].getPosition(),
                        modules[LOC_RR].getPosition()
                });
    }

    public void addVisionToOdometry() {
        LimelightHelpers.SetRobotOrientation(Constants.Limelight.limelightID,
                poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        

        LimelightHelpers.PoseEstimate lime1 = LimelightHelpers
                .getBotPoseEstimate_wpiBlue(Constants.Limelight.limelightID); // We use MegaTag 1 because 2 has problems
        addLimeLightMeasurementToPoseEstimation(lime1);

    }

    private void addLimeLightMeasurementToPoseEstimation(LimelightHelpers.PoseEstimate lime) {
        if (lime == null)
            return;

        // TODO: tune these values
        final double farDist = 2;
        final double maxRotationSpeed = 350;

        final boolean isRobotSpinningFast = Math
                .abs(RobotContainer.gyro.getAngularVelocityZWorld().getValueAsDouble()) > maxRotationSpeed;
        final boolean isTagInNarrowAngle = true;/*
                                                 * Math
                                                 * .abs(RobotContainer.getGyroRotation2d().getDegrees() -
                                                 * lime.pose.getRotation().getDegrees()) <= narrowAngleThreshold;
                                                 */
        if (isRobotSpinningFast || !isTagInNarrowAngle) {
            // if our angular velocity is greater than 720 degrees
            // per second, ignore vision updates
            return;
        }

        if (lime.tagCount == 0)
            return;

        if (lime.avgTagDist > farDist || lime.avgTagDist < 0.0)
            return;

        poseEstimator.setVisionMeasurementStdDevs(
                VecBuilder.fill(0.7 * (1.0 - lime.avgTagDist / farDist),
                        0.7 * (1.0 - lime.avgTagDist / farDist),
                        Units.degreesToRadians(30)));
        poseEstimator.addVisionMeasurement(
                lime.pose,
                lime.timestampSeconds);
    }

    public void resetOdoemetry(Pose2d newPose) {
        poseEstimator.resetPosition(RobotContainer.getGyroRotation2d(), getModulePositions(), newPose);
    }

    public void stopMotors() {
        for (var module : modules) {
            module.stopMotors();
        }

    }

    public void resetModulesToAbsolute() {
        for (SwerveModule module : modules) {
            module.resetToAbsolute();
        }
    }

    public void setIdleMode(IdleMode mode) {
        for (var module : modules) {
            module.setIdleMode(mode);
        }
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                modules[0].getPosition(),
                modules[1].getPosition(),
                modules[2].getPosition(),
                modules[3].getPosition(),
        };
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Chassis speed vx [mps]", () -> lastSpeeds.vxMetersPerSecond, null);
        builder.addDoubleProperty("Chassis speed vy [mps]", () -> lastSpeeds.vyMetersPerSecond, null);
        builder.addDoubleProperty("Chassis speed omega [rad p s]", () -> lastSpeeds.omegaRadiansPerSecond, null);
        builder.addDoubleProperty("pose estimator pos x [m]", () -> poseEstimator.getEstimatedPosition().getX(),
                (double posX) -> {
                    Pose2d currentPos = poseEstimator.getEstimatedPosition();
                    currentPos = new Pose2d(posX, currentPos.getY(), currentPos.getRotation());
                    poseEstimator.resetPose(currentPos);
                });
        builder.addDoubleProperty("pose estimator pos y [m]", () -> poseEstimator.getEstimatedPosition().getY(),
                (double posY) -> {
                    Pose2d currentPos = poseEstimator.getEstimatedPosition();
                    currentPos = new Pose2d(currentPos.getX(), posY, currentPos.getRotation());
                    poseEstimator.resetPose(currentPos);
                });
        builder.addDoubleProperty("pose estimator rot [deg]",
                () -> poseEstimator.getEstimatedPosition().getRotation().getDegrees(), (double rotationDeg) -> {
                    Pose2d currentPos = poseEstimator.getEstimatedPosition();
                    Rotation2d newRot = new Rotation2d(rotationDeg);
                    currentPos = new Pose2d(currentPos.getX(), currentPos.getY(), newRot);
                });
    }
}

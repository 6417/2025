package frc.robot.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.robot.RobotContainer;

public class SwerveDrive extends SubsystemBase {
    public SwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    public SwerveDrivePoseEstimator poseEstimator;

    ChassisSpeeds lastSpeeds = new ChassisSpeeds();

    public static final int LOC_FL = 0;
    public static final int LOC_FR = 1;
    public static final int LOC_RL = 2;
    public static final int LOC_RR = 3;

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

        kinematics = new SwerveDriveKinematics(
                configs[0].moduleOffset,
                configs[1].moduleOffset,
                configs[2].moduleOffset,
                configs[3].moduleOffset);

        poseEstimator = new SwerveDrivePoseEstimator(kinematics,
                RobotContainer.getGyroRotation2d(),
                getModulePositions(),
                new Pose2d(new Translation2d(0, 0), Rotation2d.fromRotations(0)));

        setDefaultCommand(new DriveCommand(this));
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(moduleStates[i]);
        }
        lastSpeeds = speeds;
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

    public void updateOdometry() {
        poseEstimator.update(
                RobotContainer.getGyroRotation2d(),
                getModulePositions());
    }

    public void resetOdoemetry(Pose2d newPose) {
        RobotContainer.gyro.reset();
        poseEstimator.resetPosition(RobotContainer.getGyroRotation2d(), getModulePositions(), newPose);
    }

    public void stopMotors() {
        for (var module : modules) {
            module.stopMotors();
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
        builder.addDoubleProperty("pose estimator pos x [m]", () -> poseEstimator.getEstimatedPosition().getX(), null);
        builder.addDoubleProperty("pose estimator pos y [m]", () -> poseEstimator.getEstimatedPosition().getY(), null);
        builder.addDoubleProperty("pose estimator rot [deg]",
                () -> poseEstimator.getEstimatedPosition().getRotation().getDegrees(), null);
    }
}

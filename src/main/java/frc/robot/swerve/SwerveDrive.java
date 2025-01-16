package frc.robot.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SwerveDrive extends SubsystemBase {
    public SwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    public SwerveDriveOdometry odometry;

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
        odometry = new SwerveDriveOdometry(kinematics, getGyroRotation2d(), getModulePositions());
        setDefaultCommand(new DriveCommand(this));

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            config = null;
        }

        AutoBuilder.configure(
                this::getPose,
                this::resetOdoemetry,
                this::getChassisSpeeds,
                (speeds, feedforwards) -> setChassisSpeeds(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0),
                        new PIDConstants(5.0, 0.0, 0.0)),
                config,
                () -> {

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
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
        return odometry.getPoseMeters();
    }

    public void updateOdometry() {
        odometry.update(
                getGyroRotation2d(),
                getModulePositions());
    }

    public void resetOdoemetry(Pose2d newPose) {
        RobotContainer.gyro.reset();
        odometry.resetPosition(getGyroRotation2d(), getModulePositions(), newPose);
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

    public Rotation2d getGyroRotation2d() {
        double inverted = Constants.SwerveDrive.isGyroInverted ? -1 : 1;
        double angle = SwerveModule.normalizeAngle(inverted * RobotContainer.gyro.getAngle() * Math.PI / 180.0);
        return Rotation2d.fromRadians(angle);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Chassis speed vx [mps]", () -> lastSpeeds.vxMetersPerSecond, null);
        builder.addDoubleProperty("Chassis speed vy [mps]", () -> lastSpeeds.vyMetersPerSecond, null);
        builder.addDoubleProperty("Chassis speed omega [rad p s]", () -> lastSpeeds.omegaRadiansPerSecond, null);
        builder.addDoubleProperty("odometry pos x [m]", () -> odometry.getPoseMeters().getX(), null);
        builder.addDoubleProperty("odometry pos y [m]", () -> odometry.getPoseMeters().getY(), null);
        builder.addDoubleProperty("odometry rot [deg]", () -> odometry.getPoseMeters().getRotation().getDegrees(),
                null);
    }
}

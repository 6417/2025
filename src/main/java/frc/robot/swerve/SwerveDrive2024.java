package frc.robot.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.function.Consumer;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.utils.Algorithms;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.swerve.commands.DriveCommand2024;

public class SwerveDrive2024 extends SubsystemBase {
    public enum MountingLocations {
        FrontRight, FrontLeft, BackRight, BackLeft
    }

    private Map<MountingLocations, SwerveModule> modules = new HashMap<>();
    private ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();
    private SwerveKinematics<MountingLocations> kinematics;

    public SwerveDrive2024() {
        setUpSwerveModules();
        zeroRelativeEncoders();
        setDefaultCommand(new DriveCommand2024());

        Map<MountingLocations, Translation2d> mountingPoints = Constants.SwerveDrive.Swerve2024.swerveModuleConfigs
                .entrySet().stream().map(Algorithms.mapEntryFunction(config -> config.mountingPoint))
                .collect(Collectors.toMap(Entry::getKey, Entry::getValue));
        kinematics = new SwerveKinematics<MountingLocations>(mountingPoints);
    }

    @Override
    public void periodic() {
        // if (Config.drive() == this) {
        // poseEstimator.update();
        // }
        System.out.println(getModulePositions()[0]);
    }

    public void zeroRelativeEncoders() {
        forEachModuleEntry(moduleEntry -> moduleEntry.getValue().zeroRelativeEncoder());
    }

    public void zeroAbsoluteEncoders() {
        forEachModuleEntry(moduleEntry -> moduleEntry.getValue().zeroAbsoluteEncoder());
    }

    private void setUpSwerveModules() {
        for (var locationConfigPair : Constants.SwerveDrive.Swerve2024.swerveModuleConfigs.entrySet()) {
            modules.put(locationConfigPair.getKey(), new SwerveModule(locationConfigPair.getValue()));
        }
        forEachModuleEntry(moduleEntry -> Shuffleboard.getTab("Drive")
                .add("SwerveModule" + moduleEntry.getKey().toString(), moduleEntry.getValue()));
    }

    public boolean isModuleZeroed(MountingLocations mountingLocation) {
        return true;
    }

    public void withModule(MountingLocations mountingLocation, Consumer<SwerveModule> consumer) {
        consumer.accept(modules.get(mountingLocation));
    }

    // private double getMaxSpeed(Map<MountingLocations, SwerveModuleState> states)
    // {
    // return states.values().stream().max(Comparator.comparing(state ->
    // state.speedMetersPerSecond))
    // .get().speedMetersPerSecond;
    // }

    // private Map<MountingLocations, SwerveModuleState> normalizeStates(
    // Map<MountingLocations, SwerveModuleState> states) {
    // if (getMaxSpeed(states) >
    // Constants.SwerveDrive.Swerve2024.maxVelocity.in(MetersPerSecond) *
    // Controls.getAccelerationSensitivity())
    // return states.entrySet().stream()
    // .map(Algorithms.mapEntryFunction(
    // Algorithms.mapSwerveModuleStateSpeed(speed -> speed / getMaxSpeed(states))))
    // .map(Algorithms.mapEntryFunction(
    // Algorithms.mapSwerveModuleStateSpeed(speed -> speed *
    // Controls.getAccelerationSensitivity())))
    // .collect(Collectors.toMap(Entry::getKey, Entry::getValue));
    // return states;
    // }

    public double percent2rotationVelocity(double val) {
        return RobotContainer.controls.maxTurnSpeed * val;
    }

    public double percent2driveVelocity(double x) {
        return RobotContainer.controls.maxVelocity * x;
    }

    public void drive(double vxPercent, double vyPercent, double rotPercent) {
        var requestedMovement = new ChassisSpeeds(
                percent2driveVelocity(vxPercent), percent2driveVelocity(vyPercent),
                percent2rotationVelocity(rotPercent));
        drive(requestedMovement);
    }

    public void drive(ChassisSpeeds requestedMovement) {
        currentChassisSpeeds = requestedMovement;
        Map<MountingLocations, SwerveModuleState> states = kinematics.toLabledSwerveModuleStates(currentChassisSpeeds);
        // states = normalizeStates(states);

        states.entrySet().forEach(
                (Entry<MountingLocations, SwerveModuleState> labeledState) -> modules
                        .get(labeledState.getKey()).setDesiredState(labeledState.getValue()));

        forEachModule(module -> module.driveForward(RobotContainer.controls.getAccelerationSensitivity()));
    }

    public void rotateAllModules(double speed) {
        forEachModule(module -> module.rotate(speed));
    }

    public void setRotationToHome(MountingLocations moduleLocation) {
        modules.get(moduleLocation).setCurrentRotationToEncoderHome();
    }

    public void forEachModule(Consumer<SwerveModule> consumer) {
        modules.values().stream().forEach(consumer);
    }

    public void stopAllMotors() {
        forEachModule(module -> module.stopAllMotors());
    }

    public void forEachModuleEntry(
            Consumer<Map.Entry<MountingLocations, SwerveModule>> consumer) {
        modules.entrySet().stream().forEach(consumer);
    }

    public void setSpeedFactor(double speedFactor) {
        assert speedFactor > 0.0 : "speedFactor must be grater than zero";
    }

    public void setIdleMode(IdleMode mode) {
        forEachModule(m -> m.setIdleMode(mode));
    }

    public SwerveModulePosition[] getModulePositions() {
        return modules.values().stream().map(SwerveModule::getOdometryPos).toArray(SwerveModulePosition[]::new);
    }

    public double getSwerveWheelSpeeds() {
        return modules.get(MountingLocations.FrontLeft).getWheelSpeed();
    }

    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Invert drive orientation",
                () -> Constants.SwerveDrive.navxPitchOffset == 180, val -> {
                    if (Constants.SwerveDrive.navxPitchOffset == 180) {
                        Constants.SwerveDrive.navxPitchOffset = 0;
                    } else {
                        Constants.SwerveDrive.navxPitchOffset = 180;
                    }
                });
    }
}

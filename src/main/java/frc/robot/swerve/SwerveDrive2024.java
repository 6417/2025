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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.utils.Algorithms;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.abstraction.baseClasses.BSwerveDrive;
import frc.robot.abstraction.baseClasses.BSwerveModule;
import frc.robot.swerve.commands.DriveCommand2024;

public class SwerveDrive2024 extends BSwerveDrive {

    private Map<MountingLocations, BSwerveModule> modules = new HashMap<>();
    private ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();

    public SwerveDrive2024() {
    }

    @Override
    public void init() {
        super.init();
        setUpSwerveModules();
        zeroRelativeEncoders();
        setDefaultCommand(new DriveCommand2024());
    }

    @Override
    public void periodic() {
        // if (Config.drive() == this) {
        // poseEstimator.update();
        // }
    }

    @Override
    public void zeroRelativeEncoders() {
        forEachModuleEntry(moduleEntry -> moduleEntry.getValue().zeroRelativeEncoder());
    }

    @Override
    public void zeroAbsoluteEncoders() {
        forEachModuleEntry(moduleEntry -> moduleEntry.getValue().zeroAbsoluteEncoder());
    }

    private void setUpSwerveModules() {
        modules = Constants.SwerveDrive.Swerve2024.swerveModuleConfigs.entrySet().stream()
                .map(Algorithms.mapEntryFunction(SwerveModule::new))
                .collect(Collectors.toMap(Entry::getKey, Entry::getValue));
        forEachModuleEntry(moduleEntry -> Shuffleboard.getTab("Drive")
                .add("SwerveModule" + moduleEntry.getKey().toString(), moduleEntry.getValue()));
    }

    @Override
    public boolean isModuleZeroed(MountingLocations mountingLocation) {
        return true;
    }

    @Override
    public void withModule(MountingLocations mountingLocation, Consumer<BSwerveModule> consumer) {
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

    @Override
    public void drive(double vxPercent, double vyPercent, double rotPercent) {
        var requestedMovement = new ChassisSpeeds(
                percent2driveVelocity(vxPercent), percent2driveVelocity(vyPercent),
                percent2rotationVelocity(rotPercent));
        drive(requestedMovement);
    }

    @Override
    public void drive(ChassisSpeeds requestedMovement) {
        currentChassisSpeeds = requestedMovement;
        Map<MountingLocations, SwerveModuleState> states = kinematics
                .toLabledSwerveModuleStates(currentChassisSpeeds);
        // states = normalizeStates(states);

        states.entrySet().forEach(
                (Entry<MountingLocations, SwerveModuleState> labeledState) -> modules
                        .get(labeledState.getKey()).setDesiredState(labeledState.getValue()));

        forEachModule(module -> module.driveForward(Controls.getAccelerationSensitivity()));
    }

    @Override
    public void rotateAllModules(double speed) {
        forEachModule(module -> module.rotate(speed));
    }

    @Override
    public void setRotationToHome(MountingLocations moduleLocation) {
        modules.get(moduleLocation).setCurrentRotationToEncoderHome();
    }

    @Override
    public void forEachModule(Consumer<BSwerveModule> consumer) {
        modules.values().stream().forEach(consumer);
    }

    @Override
    public void stopAllMotors() {
        forEachModule(module -> module.stopAllMotors());
    }

    @Override
    public void forEachModuleEntry(
            Consumer<Map.Entry<MountingLocations, BSwerveModule>> consumer) {
        modules.entrySet().stream().forEach(consumer);
    }

    @Override
    public void setSpeedFactor(double speedFactor) {
        assert speedFactor > 0.0 : "speedFactor must be grater than zero";
    }

    @Override
    public void setVolts(double leftvolts, double rigthvolts) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVolts'");
    }

    @Override
    public void driveToPos(Pose2d pos) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'driveToPos'");
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        forEachModule(m -> m.setIdleMode(mode));
    }

    @Override
    public Command sysIdQuasistatic(Direction direction) {
        throw new UnsupportedOperationException("Unimplemented method 'sysIdQuasistatic'");
    }

    @Override
    public Command sysIdDynamic(Direction direction) {
        throw new UnsupportedOperationException("Unimplemented method 'sysIdDynamic'");
    }

    // TODO: abstraction over motorrole
    @Override
    public <T> FridolinsMotor getMotor(T motor) {
        assert motor instanceof BSwerveDrive.SwerveMotor;
        var swerveMotor = (BSwerveDrive.SwerveMotor) motor;
        return switch (swerveMotor.getMotorType()) {
            case DRIVE -> modules.get(swerveMotor.getLocation()).getDriveMotor();
            case ROTATE -> modules.get(swerveMotor.getLocation()).getRotationMotor();
        };
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        return modules.values().stream().map(BSwerveModule::getOdometryPos).toArray(SwerveModulePosition[]::new);
    }

    @Override
    public double getSwerveWheelSpeeds() {
        return modules.get(MountingLocations.FrontLeft).getWheelSpeed();
    }

    @Override
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

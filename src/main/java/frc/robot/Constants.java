// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.ctre.phoenix6.configs.Slot0Configs;
import frc.fridowpi.motors.utils.FeedForwardValues;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.fridowpi.joystick.IJoystickId;
import frc.fridowpi.motors.FridoFalcon500v6;
import frc.fridowpi.motors.FridoTalonSRX;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.FridoFeedBackDevice;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.FridolinsMotor.LimitSwitchPolarity;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.abstraction.baseClasses.BDrive.MountingLocations;
import frc.robot.abstraction.baseClasses.BSwerveDrive.MotorType;
import frc.robot.swerve.SwerveModule;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Joystick {
        public static final IJoystickId primaryJoystickId = () -> 0;
        public static final IJoystickId secondaryJoystickId = () -> 1;
        public static final int idCounterStart = 1000;
        public static final double lt_rt_reshold = 0.2;
    }

    public static final class SwerveDrive {

        public static double navxPitchOffset = 90;

        public static final class Swerve2024 {

            public static final double gearRatio = 1 / 5.192308;

            public static final boolean enabled = true;
            public static final double absoluteEncoderZeroPositionTolerance = 0.01;
            public static final boolean joystickYinverted = true;
            public static final boolean joystickXinverted = true;
            public static final double deadBand = 0.08;
            public static final double yOffsetMapperMaxVoltage = 12.5;
            public static final double yOffsetMapperMinVoltage = 9;

            public static final double modulesXOffset = 0.275;
            public static final double modulesYOffset = 0.275;

            public static final double maxVelocity = 5.2;
            // Not yet tested
            public static final double maxAcceleration = 1;
            public static final double maxTurnSpeed = maxVelocity // v (m/x)
                    / Math.hypot(modulesXOffset, modulesYOffset) // radius of the circle described by motors
                                                                 // during the turn movement
                    * 2 * Math.PI; // 2 * pi * radius

            public static final double absoluteEncoderToMeters = 1;
            public static final double metersToRelativeEncoder = 1;

            private static final List<Integer> motorIds = List.of(
                    1, 2, 3, 4,
                    11, 12, 13, 14);

            public static final Map<MountingLocations, frc.robot.swerve.SwerveModule.Config> swerveModuleConfigs = new HashMap<>();

            public static frc.robot.swerve.SwerveModule.Config commonConfigurations = new frc.robot.swerve.SwerveModule.Config();

            static {
                addCommonModuleConfigurarions();
                addModuleSpecificConfigurarions();
            }

            private static void addCommonModuleConfigurarions() {
                commonConfigurations.driveMotorTicksPerRotation = 1;
                commonConfigurations.rotationMotorTicksPerRotation = 47.691;
                commonConfigurations.drivePID = new PidValues(0.1, 0, 0);
                commonConfigurations.driveFeedForward = new FeedForwardValues(0.39, 0.17);
                commonConfigurations.driveAccelerationLimitRotationsPerS2 = 100;
                commonConfigurations.drivePID.slotIdX = Optional.of(0);
                commonConfigurations.rotationPID = new PidValues(1.05, 0.01, 1);
                commonConfigurations.rotationPID.slotIdX = Optional.of(0);
                commonConfigurations.wheelCircumference = 0.12 * Math.PI;
                commonConfigurations.maxVelocity = maxVelocity;
                commonConfigurations.driveEncoderType = FridoFeedBackDevice.kBuildin;
                commonConfigurations.rotationEncoderType = FridoFeedBackDevice.kBuildin;
                commonConfigurations.driveIdleMode = IdleMode.kCoast;
                commonConfigurations.rotationIdleMode = IdleMode.kBrake;
            }

            public static final double xOffset = modulesXOffset;
            public static final double yOffset = modulesYOffset;

            private static FridolinsMotor driveMotorInitializer(int id) {
                var motor = new FridoFalcon500v6(id);
                motor.factoryDefault();
                motor.asTalonFX().getConfigurator()
                        .apply(new Slot0Configs().withKP(0.03).withKS(0.18).withKV(0.27));
                return motor;
            }

            private static FridolinsMotor angleMotorInitializer(int id) {
                var motor = new FridoCanSparkMax(id, MotorType.kBrushless);
                motor.factoryDefault();
                motor.setSmartCurrentLimit(20, 20);
                motor.getPIDController().setIZone(1.5);
                return motor;
            }

            public static final Translation2d[] SWERVE_MODULE_TRANSLATIONS = {
                    new Translation2d(xOffset, yOffset), // FL
                    new Translation2d(xOffset, -yOffset), // FR
                    new Translation2d(-xOffset, yOffset), // BL
                    new Translation2d(-xOffset, -yOffset), // BR
            };

            public static final SwerveModulePosition[] SWERVE_MODULE_POSITIONS = {
                    new SwerveModulePosition(SWERVE_MODULE_TRANSLATIONS[0].getNorm(),
                            SWERVE_MODULE_TRANSLATIONS[0].getAngle()),
                    new SwerveModulePosition(SWERVE_MODULE_TRANSLATIONS[1].getNorm(),
                            SWERVE_MODULE_TRANSLATIONS[1].getAngle()),
                    new SwerveModulePosition(SWERVE_MODULE_TRANSLATIONS[2].getNorm(),
                            SWERVE_MODULE_TRANSLATIONS[2].getAngle()),
                    new SwerveModulePosition(SWERVE_MODULE_TRANSLATIONS[3].getNorm(),
                            SWERVE_MODULE_TRANSLATIONS[3].getAngle())
            };

            private static void addModuleSpecificConfigurarions() {

                SwerveModule.Config frontLeftConfig = commonConfigurations
                        .clone();
                frontLeftConfig.absoluteEncoderZeroPosition = 0.150;
                // frontLeftConfig.mountingPoint = SWERVE_MODULE_TRANSLATIONS[0];
                frontLeftConfig.mountingPoint = new Translation2d(-xOffset, yOffset);
                frontLeftConfig.driveMotorInitializer = () -> driveMotorInitializer(1);
                frontLeftConfig.rotationMotorInitializer = () -> angleMotorInitializer(11);
                frontLeftConfig.driveMotorInverted = false;
                frontLeftConfig.absoluteEncoderChannel = 0;
                swerveModuleConfigs.put(MountingLocations.FrontLeft, frontLeftConfig);

                SwerveModule.Config frontRightConfig = commonConfigurations
                        .clone();
                frontRightConfig.absoluteEncoderZeroPosition = 1.0 - 0.221 - 0.05;
                // frontLeftConfig.mountingPoint = SWERVE_MODULE_TRANSLATIONS[1];
                frontRightConfig.mountingPoint = new Translation2d(-xOffset, -yOffset);
                frontRightConfig.driveMotorInitializer = () -> driveMotorInitializer(2);
                frontRightConfig.rotationMotorInitializer = () -> angleMotorInitializer(12);
                frontRightConfig.driveMotorInverted = false;
                frontRightConfig.absoluteEncoderChannel = 1;
                swerveModuleConfigs.put(MountingLocations.FrontRight, frontRightConfig);

                SwerveModule.Config backLeftConfig = commonConfigurations
                        .clone();
                backLeftConfig.absoluteEncoderZeroPosition = 0.487;
                // frontLeftConfig.mountingPoint = SWERVE_MODULE_TRANSLATIONS[2];
                backLeftConfig.mountingPoint = new Translation2d(xOffset, yOffset);
                backLeftConfig.driveMotorInitializer = () -> driveMotorInitializer(3);
                backLeftConfig.rotationMotorInitializer = () -> angleMotorInitializer(13);
                backLeftConfig.driveMotorInverted = false;
                backLeftConfig.absoluteEncoderChannel = 2;
                swerveModuleConfigs.put(MountingLocations.BackLeft, backLeftConfig);

                SwerveModule.Config backRightConfig = commonConfigurations
                        .clone();
                backRightConfig.absoluteEncoderZeroPosition = 0.480;
                backRightConfig.mountingPoint = new Translation2d(xOffset, -yOffset);
                backRightConfig.driveMotorInitializer = () -> driveMotorInitializer(4);
                backRightConfig.rotationMotorInitializer = () -> angleMotorInitializer(14);
                backRightConfig.driveMotorInverted = false;
                backRightConfig.absoluteEncoderChannel = 3;
                swerveModuleConfigs.put(MountingLocations.BackRight, backRightConfig);
            }

            public static final class Swerve2019 {
                private static void setSwerveDriveConstants() {
                    zeroingSpeed = 500;
                    maxFineTuneOffsetForZeroEncodersCommand = 196608 / 100;
                    maxSpeedOfDrive = 1;
                }

                public static final double gearRatio = 1.0 / 5.192308;

                public static final boolean enabled = false;
                public static final boolean rotateAllModulesInSameDirection = false;
                public static final boolean joystickYinverted = true;
                public static final boolean joystickXinverted = true;
                public static double zeroingSpeed;
                public static final double deadBand = 0.015;
                public static final double yOffsetMapperMaxVoltage = 12.5;
                public static final double yOffsetMapperMinVoltage = 9;
                public static final double finetuningZeroFactor = 0.1;
                public static double maxFineTuneOffsetForZeroEncodersCommand;
                public static double maxSpeedOfDrive; // in meters per second
                public static final double maxRotationSpeed = 15 * Math.PI / 16; // at full rotation speed the robot
                                                                                 // will
                                                                                 // turn
                                                                                 // by 180 degrees, in rad per second
                public static final Map<MountingLocations, SwerveModule.Config> swerveModuleConfigs = new HashMap<>();

                public static SwerveModule.Config commonConfigurations = new SwerveModule.Config();
                public static double defaultSpeedFactor = 0.3;
                public static double slowSpeedFactor = 0.35;
                public static double fullSpeed = 1.0;

                static {
                    setSwerveDriveConstants();
                    addCommonModuleConfigurarions();
                    addModuleSpecificConfigurarions();
                }

                private static void addCommonModuleConfigurarions() {
                    commonConfigurations.driveMotorTicksPerRotation = 11_564.0;
                    commonConfigurations.rotationMotorTicksPerRotation = 196_608.0;
                    commonConfigurations.drivePID = new PidValues(0.015, 0.0, 0.0, 0.03375);
                    commonConfigurations.drivePID.slotIdX = Optional.of(0);
                    commonConfigurations.rotationPID = new PidValues(0.04, 0.0, 0.5);
                    commonConfigurations.rotationPID.slotIdX = Optional.of(0);
                    commonConfigurations.wheelCircumference = 0.1 * Math.PI;
                    commonConfigurations.maxVelocity = maxSpeedOfDrive;
                    commonConfigurations.driveEncoderType = FridoFeedBackDevice.kRelative;
                    commonConfigurations.rotationEncoderType = FridoFeedBackDevice.kRelative;
                    commonConfigurations.limitSwitchPolarity = LimitSwitchPolarity.kNormallyOpen;
                }

                private static FridoTalonSRX angleMotorInitializer(int id) {
                    FridoTalonSRX motor = new FridoTalonSRX(id);
                    motor.factoryDefault();
                    return motor;
                }

                private static FridolinsMotor driveMotorInitializer(int id) {
                    FridoTalonSRX motor = angleMotorInitializer(id, MotorType.kBrushless);
                    motor.enableForwardLimitSwitch(LimitSwitchPolarity.kDisabled, false);
                    motor.enableReverseLimitSwitch(LimitSwitchPolarity.kDisabled, false);
                    return motor;
                }

                private static void addModuleSpecificConfigurarions() {
                    // TODO: Add limit switch stuff again

                    SwerveModule.Config frontLeftConfig = commonConfigurations.clone();
                    frontLeftConfig.mountingPoint = new Translation2d(0.32, 0.305);
                    frontLeftConfig.driveMotorInitializer = () -> driveMotorInitializer(32);
                    frontLeftConfig.rotationMotorInitializer = () -> angleMotorInitializer(33);
                    swerveModuleConfigs.put(MountingLocations.FrontLeft, frontLeftConfig);

                    SwerveModule.Config frontRightConfig = commonConfigurations.clone();
                    frontRightConfig.mountingPoint = new Translation2d(-0.32, 0.305);
                    frontRightConfig.driveMotorInitializer = () -> driveMotorInitializer(38);
                    frontRightConfig.rotationMotorInitializer = () -> angleMotorInitializer(39);
                    swerveModuleConfigs.put(MountingLocations.FrontRight, frontRightConfig);

                    SwerveModule.Config backLeftConfig = commonConfigurations.clone();
                    backLeftConfig.mountingPoint = new Translation2d(0.32, -0.305);
                    backLeftConfig.driveMotorInitializer = () -> driveMotorInitializer(34);
                    backLeftConfig.rotationMotorInitializer = () -> angleMotorInitializer(35);
                    swerveModuleConfigs.put(MountingLocations.BackLeft, backLeftConfig);

                    SwerveModule.Config backRightConfig = commonConfigurations.clone();
                    backRightConfig.mountingPoint = new Translation2d(-0.32, -0.305);
                    backRightConfig.driveMotorInitializer = () -> driveMotorInitializer(36);
                    backRightConfig.rotationMotorInitializer = () -> angleMotorInitializer(37);
                    swerveModuleConfigs.put(MountingLocations.BackRight, backRightConfig);
                }
            }
        }
    }
}

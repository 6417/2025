// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.fridowpi.motors.utils.FeedForwardValues;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.fridowpi.motors.FridolinsMotor.LimitSwitchPolarity;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.subsystems.CoralDispenserSubsystem;
import frc.robot.swerve.ModuleConfig;

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
        public static final int driveJoystickId = 0;
        public static final int operatorJoystickId = 1;
        public static final int idCounterStart = 1000;
        public static final double lt_rt_reshold = 0.2;
    }

    public static final class Gyro {
        public static final int gyroId = 0;
    }

    public static final class LEDs {
        public static final int ledPortLeft = 42;
        public static final int ledPortRight = 42;
        public static final int ledBufferLength = 60;
    }

    public static final class Autonomous {
        public static final String autoGroup = "AutoPathLeftStart";
    }

    public static final class Limelight {
        public static final String limelightID = "limelight-vier";

        public static final List<Double> aprilTagsForOuttakeStateTeamIsRed = Arrays.asList(17.0, 18.0, 19.0, 20.0, 21.0,
                22.0);
        public static final List<Double> aprilTagsForIntakeStateTeamIsRed = Arrays.asList(1.0, 2.0);

        public static final List<Double> aprilTagsForOuttakeStateTeamIsBlue = Arrays.asList(6.0, 7.0, 8.0, 9.0, 10.0,
                11.0);
        public static final List<Double> aprilTagsForIntakeStateTeamIsBlue = Arrays.asList(12.0, 13.0);
    }

    public static final class OffsetsToAprilTags {
        // This values must be calibrated to value that we can both score and see the
        // april tag.(And obviously when we are in allience zone)
        public static final double[] offsetToAprilTagLoadingStation = { 0.45, 0.0 + 0, 0.0 };
        public static final double[] offsetToAprilTagRight = { 0.55, 0.2 + 0.05, 0 };
        public static final double[] offsetToAprilTagLeft = { 0.55, -0.2 + 0.13, 0 };
    }

    public static final class CoralDispenser {
        public static final int CoralEndEffectorMotorID = 50;
        public static final int coralPitchMotorID = 40;
        public static final LimitSwitchPolarity revPolarity = LimitSwitchPolarity.kNormallyOpen;
        public static final LimitSwitchPolarity fwdPolarity = LimitSwitchPolarity.kNormallyOpen;
        public static final LimitSwitchPolarity fwdMotorTopPolarity = LimitSwitchPolarity.kNormallyOpen;
        public static final double pitchMotorForwardLimit = 68;
        public static final double pitchMotorReverseLimit = 10;
        public static final double zeroingSpeed = 0.1;
        public static final double kArmGearRatio = 50 * (37 / 9);
        public static final double angularOffset = 0; // It is setted on REVClient

        public static final double stopSpeedMotorTop = 0;
        public static final double stopSpeedPitch = 0;

        public static double kMaxAcceleration = 50000;
        public static double kMaxVelocity = 8000; // in rpm
        public static double kAllowedClosedLoopError = 1;

        public static final int stationState = 0;
        public static final int l1State = 1;
        public static final int l2State = 2;
        public static final int l3State = 3;
        public static final int l4State = 4;
        public static final int algae1State = 5;
        public static final int algae2State = 6;
        public static final int steadyState = 7;


        public static final double pitchUp = 10;
        public static final double steadyStateSetpoint = 20;
        public static final double pitchDown = 70;

        public static final PidValues PidValuesPitch = new PidValues(0.005, 0, 0.005, 0);
        public static final double intakeSpeed = -0.8;
        public static final double outtakeSpeed = 0.6;

        public static final double waitAfterOuttake = 0.3;
        public static final double waitAfterAlgaeIntake = 0.3;
    }

    public static final class LevelParameters implements Sendable {
        public String name;
        public double pitchAngle;
        public double height;

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty("pitch angle [ticks]", () -> pitchAngle,
                    (double angle) -> pitchAngle = angle);
            builder.addDoubleProperty("height [m]", () -> height, (double h) -> height = h);
        }
    }

    public static LevelParameters[] parameters = new LevelParameters[8];

    static {
        for (int i = 0; i < parameters.length; i++)
            parameters[i] = new LevelParameters();

        parameters[CoralDispenser.stationState].name = "station";
        parameters[CoralDispenser.l1State].name = "l1";
        parameters[CoralDispenser.l2State].name = "l2";
        parameters[CoralDispenser.l3State].name = "l3";
        parameters[CoralDispenser.l4State].name = "l4";
        parameters[CoralDispenser.steadyState].name = "Steady State";
        parameters[CoralDispenser.algae1State].name = "Algae1";
        parameters[CoralDispenser.algae2State].name = "Algae2";

        parameters[CoralDispenser.stationState].pitchAngle = 27;
        parameters[CoralDispenser.l1State].pitchAngle = 60;
        parameters[CoralDispenser.l2State].pitchAngle = 50;
        parameters[CoralDispenser.l3State].pitchAngle = 47;
        parameters[CoralDispenser.l4State].pitchAngle = 60;
        parameters[CoralDispenser.steadyState].pitchAngle = 20;
        parameters[CoralDispenser.algae1State].pitchAngle = 43;
        parameters[CoralDispenser.algae2State].pitchAngle = 43;

        parameters[CoralDispenser.stationState].height = 1;
        parameters[CoralDispenser.l1State].height = 3;
        parameters[CoralDispenser.l2State].height = 8;
        parameters[CoralDispenser.l3State].height = 29;
        parameters[CoralDispenser.l4State].height = 72.5;
        parameters[CoralDispenser.steadyState].height = 3;
        parameters[CoralDispenser.algae1State].height = 5;
        parameters[CoralDispenser.algae2State].height = 27;
    }

    public static final class ClimberSubsystem {
        public static final int climberMotorID = 20;

        public static final double resetPitchEncoderPosition = 0;
        public static PidValues PidValuesOutClimberSubsystem = new PidValues(0.05, 0, 0.6, 0);
        public static PidValues PidValuesInClimberSubsystem = new PidValues(0.05, 0, 0.2, 0);

        public static double kAllowedClosedLoopErrorOut = 0.5;
        public static double kMaxAccelerationOut = 30000;
        public static double kMaxVelocityOut = 3000; // in rpm
        // TODO: Remove two different PID because we dont need it
        public static double kAllowedClosedLoopErrorIn = 0.5;
        public static double kMaxAccelerationIn = 60000;
        public static double kMaxVelocityIn = 6000;

        public static double positionFront = 235;
        public static double positionBack = 70; // Adjusted to prevent robot to tip over wegen des Schwerpunkts -
                                                // Wiedercalibration wird angefordert
        public static double positionSteady = 180;
    }

    public static final class LiftingTower {
        public static final int liftingTowerLeftId = 30;
        public static final int liftingTowerRightId = 31;

        public static final double zeroingSpeed = -0.1;
        public static final double resetEncoderPosition = 0;
        public static final LimitSwitchPolarity towerBottomSwitchPolarity = LimitSwitchPolarity.kNormallyClosed;

        public static final double kMaxVelocity = 3000;
        public static final double kMaxAcceleration = 8000;
        public static final double kAllowedClosedLoopError = 0.01;
        public static final PidValues pidValues = new PidValues(2.0, 0.0, 0.0, 0); // TODO: test all values

        static {
            pidValues.iZone = Optional.of(4.0);
        }
        public static final double softLimitTopPos = 72.5;
        public static final double softLimitBottomPos = 1.0;
    }

    public static final class SwerveDrive {
        public static ModuleConfig[] configs = new ModuleConfig[4];
        public static boolean isGyroInverted = false;

        public static final double maxSpeed = 4.9; // TODO: for testing
        public static ModuleConfig defaultModuleConfig2024 = new ModuleConfig();
        public static final double moduleXoffset = 0.267;
        public static final double moduleYoffset = 0.267;
        public static final double maxTurnSpeed = 10;// 12// Math.hypot(moduleXoffset, moduleYoffset) * maxSpeed /
                                                     // (Math.PI *
                                                     // 2); // rps

        static {
            defaultModuleConfig2024.maxSpeed = maxSpeed;
            defaultModuleConfig2024.wheelCircumference = Units.inchesToMeters(4) * Math.PI * 0.977 * 1.058376;

            defaultModuleConfig2024.driveGearboxRatio = 6.181;
            defaultModuleConfig2024.driveMotorStallCurrentLimit = 70;
            defaultModuleConfig2024.driveMotorFreeCurrentLimit = 25;
            defaultModuleConfig2024.drivePidValues = new PidValues(1.1926E-05, 0.00, 0);
            //defaultModuleConfig2024.driveFFValues = new FeedForwardValues(0.13271 / 12, 2.1395 / 12, 0.15313 / 12);
            defaultModuleConfig2024.driveFFValues = new FeedForwardValues(0.057, 0.1177, 0.05);

            defaultModuleConfig2024.angleGearboxRatio = 7.44;
            defaultModuleConfig2024.angleMotorStallCurrentLimit = 35;
            defaultModuleConfig2024.angleMotorFreeCurrentLimit = 20;
            defaultModuleConfig2024.angleMotorIzone = 0.1;
            defaultModuleConfig2024.anglePidValues = new PidValues(0.4, 0.0, 0.05);

            defaultModuleConfig2024.encoderThicksToRotationFalcon = 1;
            defaultModuleConfig2024.encoderVelocityToRPSFalcon = 1;
            defaultModuleConfig2024.encoderThicksToRotationNEO = 1;
            defaultModuleConfig2024.encoderVelocityToRPSNEO = 1;

            final int LOC_FL = frc.robot.swerve.SwerveDrive.LOC_FL;
            final int LOC_FR = frc.robot.swerve.SwerveDrive.LOC_FR;
            final int LOC_RL = frc.robot.swerve.SwerveDrive.LOC_RL;
            final int LOC_RR = frc.robot.swerve.SwerveDrive.LOC_RR;

            configs[LOC_FL] = defaultModuleConfig2024.clone();
            configs[LOC_FR] = defaultModuleConfig2024.clone();
            configs[LOC_RL] = defaultModuleConfig2024.clone();
            configs[LOC_RR] = defaultModuleConfig2024.clone();

            configs[LOC_FL].driveMotorID = 1;
            configs[LOC_FL].angleMotorID = 11;
            configs[LOC_FL].driveMotorInverted = false;
            configs[LOC_FL].angleMotorInverted = true;
            configs[LOC_FL].moduleOffset = new Translation2d(moduleXoffset, moduleYoffset);
            configs[LOC_FL].encoderChannel = 0;
            configs[LOC_FL].absEncoderOffset = 0.763;

            configs[LOC_FR].driveMotorID = 2;
            configs[LOC_FR].angleMotorID = 12;
            configs[LOC_FR].driveMotorInverted = false;
            configs[LOC_FR].angleMotorInverted = true;
            configs[LOC_FR].moduleOffset = new Translation2d(moduleXoffset, -moduleYoffset);
            configs[LOC_FR].encoderChannel = 1;
            configs[LOC_FR].absEncoderOffset = 0.0857;

            configs[LOC_RL].driveMotorID = 3;
            configs[LOC_RL].angleMotorID = 13;
            configs[LOC_RL].driveMotorInverted = false;
            configs[LOC_RL].angleMotorInverted = true;
            configs[LOC_RL].moduleOffset = new Translation2d(-moduleXoffset, moduleYoffset);
            configs[LOC_RL].encoderChannel = 2;
            configs[LOC_RL].absEncoderOffset = 0.446;

            configs[LOC_RR].driveMotorID = 4;
            configs[LOC_RR].angleMotorID = 14;
            configs[LOC_RR].driveMotorInverted = false;
            configs[LOC_RR].angleMotorInverted = true;
            configs[LOC_RR].moduleOffset = new Translation2d(-moduleXoffset, -moduleYoffset);
            configs[LOC_RR].encoderChannel = 3;
            configs[LOC_RR].absEncoderOffset = 0.699;
        }
    }
}

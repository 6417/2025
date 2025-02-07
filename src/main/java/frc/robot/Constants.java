// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.fridowpi.motors.utils.FeedForwardValues;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.fridowpi.motors.FridolinsMotor.LimitSwitchPolarity;
import frc.fridowpi.motors.utils.PidValues;
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

    public static final class Autonomous {
        public static final String autoGroup = "AutoPathLeftStart";
    }

    public static final class Limelight {
        public static final String limelightID = "limelight-drei";
        public static final String limelightBackID = "limelight-vier";

        public static final List<Double> aprilTagsForOuttakeStateTeamIsRed = Arrays.asList(17.0, 18.0, 19.0, 20.0, 21.0,
                22.0);
        public static final List<Double> aprilTagsForIntakeStateTeamIsRed = Arrays.asList(1.0, 2.0);

        public static final List<Double> aprilTagsForOuttakeStateTeamIsBlue = Arrays.asList(6.0, 7.0, 8.0, 9.0, 10.0,
                11.0);
        public static final List<Double> aprilTagsForIntakeStateTeamIsBlue = Arrays.asList(12.0, 13.0);
    }

    public static final class OffsetsToAprilTags {
        public static final double[] offsetToAprilTagLeftToReef = { 0.5, 0.165, 0 };
        public static final double[] offsetToAprilTagRightToReef = { 0.5, -0.165, 0 };
        public static final double[] offsetToAprilTagCenterToReef = { 0.5, 0, 0 };
    }

    public static final class CoralDispenser {
        public static final int coralMotorTopID = 40;
        public static final int coralMotorBottomID = 41;
        public static final LimitSwitchPolarity revPolarity = LimitSwitchPolarity.kNormallyOpen;
        public static final LimitSwitchPolarity fwdPolarity = LimitSwitchPolarity.kNormallyOpen;
        public static final LimitSwitchPolarity fwdMotorTopPolarity = LimitSwitchPolarity.kNormallyOpen;
        public static final double resetPitchEncoderPosition = 0;
        public static final double resetMotorTopEncoderPosition = 0;
        public static final double zeroingSpeed = 0.1;

        public static final double stopSpeedMotorTop = 0;
        public static final double stopSpeedPitch = 0;

        public static final int neutralState = 0;
        public static final int stationState = 1;
        public static final int l1State = 2;
        public static final int l2State = 3;
        public static final int l3State = 4;
        public static final int l4State = 5;
        
        public static final PidValues PidValuesPitch = new PidValues(0, 0, 0,0);
        public static final PidValues PidValuesMotorTop = new PidValues(0, 0, 0,0);
        public static final double intakeSpeed = -0.5;
        public static final double outtakeSpeed = 0.5;

        public static final double waitAfterOuttake = 0.3;
        public static final double zeroingPosition = 0;
        public static final double waitAfterAlgaeIntake = 0.3;
    }

    public static final class LevelParameters implements Sendable{
        public String name;
        public Rotation2d pitchAngle;
        public double height;
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty("pitch angle [deg]", () -> pitchAngle.getDegrees(), (double angle) -> pitchAngle = Rotation2d.fromDegrees(angle));
            builder.addDoubleProperty("height [m]", () -> height, (double h) -> height = h);
        }
    }

    public static LevelParameters[] parameters = new LevelParameters[6];

    static {
        parameters[CoralDispenser.neutralState].name = "neutral";
        parameters[CoralDispenser.stationState].name = "station";
        parameters[CoralDispenser.l1State].name = "l1";
        parameters[CoralDispenser.l2State].name = "l2";
        parameters[CoralDispenser.l3State].name = "l3";
        parameters[CoralDispenser.l4State].name = "l4";
    }


    public static final class ClimberSubsytem {
        public static final int climberMotorID = 59;
        public static final int coralMotorChangePitchID = 58;

        public static final PidValues PidValuesClimberSubsystem = new PidValues(0, 0, 0,0);
        public static final double resetPitchEncoderPosition = 0;
    }

    public static final class LiftingTower {
        public static final int liftingTowerLeftId = 30;
        public static final int liftingTowerRightId = 31;

        public static final double stopSpeed = 0;

        public static final PidValues PidValuesLiftingTower = new PidValues(0, 0, 0,0);
        public static final double resetEncoderPosition = 0;
        public static final LimitSwitchPolarity fdwLiftingTowePolarity = LimitSwitchPolarity.kNormallyOpen;
        public static final double zeroingSpeed = 0.1;
    }


    public static final class SwerveDrive {
        public static ModuleConfig[] configs = new ModuleConfig[4];
        public static boolean isGyroInverted = true;

        public static final double maxSpeed = 4.45; // TODO: for testing
        public static ModuleConfig defaultModuleConfig2024 = new ModuleConfig();
        public static final double moduleXoffset = 0.275;
        public static final double moduleYoffset = 0.275;
        public static final double maxTurnSpeed = Math.hypot(moduleXoffset, moduleYoffset) * maxSpeed / (Math.PI * 2); // rps

        static {
            defaultModuleConfig2024.maxSpeed = maxSpeed;
            defaultModuleConfig2024.wheelCircumference = Units.inchesToMeters(4) * Math.PI;

            defaultModuleConfig2024.driveGearboxRatio = 8.11;
            defaultModuleConfig2024.driveMotorStallCurrentLimit = 55;
            defaultModuleConfig2024.driveMotorFreeCurrentLimit = 30;
            defaultModuleConfig2024.drivePidValues = new PidValues(0.15, 0.00, 0);
            defaultModuleConfig2024.driveFFValues = new FeedForwardValues(0.075, 0.1125, 0);

            defaultModuleConfig2024.angleGearboxRatio = 7.44;
            defaultModuleConfig2024.angleMotorStallCurrentLimit = 35;
            defaultModuleConfig2024.angleMotorFreeCurrentLimit = 20;
            defaultModuleConfig2024.angleMotorIzone = 0.1;
            defaultModuleConfig2024.anglePidValues = new PidValues(1.1, 0.03, 0.25);

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
            configs[LOC_FL].absEncoderOffset = 0.360;

            configs[LOC_FR].driveMotorID = 2;
            configs[LOC_FR].angleMotorID = 12;
            configs[LOC_FR].driveMotorInverted = false;
            configs[LOC_FR].angleMotorInverted = true;
            configs[LOC_FR].moduleOffset = new Translation2d(moduleXoffset, -moduleYoffset);
            configs[LOC_FR].encoderChannel = 1;
            configs[LOC_FR].absEncoderOffset = 0.0144;

            configs[LOC_RL].driveMotorID = 3;
            configs[LOC_RL].angleMotorID = 13;
            configs[LOC_RL].driveMotorInverted = false;
            configs[LOC_RL].angleMotorInverted = true;
            configs[LOC_RL].moduleOffset = new Translation2d(-moduleXoffset, moduleYoffset);
            configs[LOC_RL].encoderChannel = 2;
            configs[LOC_RL].absEncoderOffset = 0.704;

            configs[LOC_RR].driveMotorID = 4;
            configs[LOC_RR].angleMotorID = 14;
            configs[LOC_RR].driveMotorInverted = false;
            configs[LOC_RR].angleMotorInverted = true;
            configs[LOC_RR].moduleOffset = new Translation2d(-moduleXoffset, -moduleYoffset);
            configs[LOC_RR].encoderChannel = 3;
            configs[LOC_RR].absEncoderOffset = 0.165;
        }
    } 
}

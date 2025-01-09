package frc.robot.swerve.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.fridowpi.sensors.FridoNavx;
import frc.fridowpi.utils.Vector2;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.RobotContainer;

/**
 * DriveCommand
 */
public class DriveCommand2024 extends Command {

    private SlewRateLimiter xLimiter = new SlewRateLimiter(RobotContainer.controls.slewRateLimit, -1000, 0);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(RobotContainer.controls.slewRateLimit, -1000, 0);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(RobotContainer.controls.slewRateLimit, -1000, 0);

    private double maxSpeed = 0;

    public DriveCommand2024() {
        addRequirements(RobotContainer.drive);
    }

    @Override
    public void execute() {
        var speed = abs(RobotContainer.drive.getSwerveWheelSpeeds());
        if (speed > Constants.SwerveDrive.Swerve2024.maxVelocity && speed > maxSpeed) {
            maxSpeed = speed;
            System.out.println("MaxSpeed: " + speed);
        }

        var pitchOffsetRadians = Radians.convertFrom(Constants.SwerveDrive.navxPitchOffset, Degrees);

        var joystick = RobotContainer.controls.driveJoystick;
        var xy = new Vector2(joystick.getX(), joystick.getY());
        xy = Vector2.fromRadians(xy.getAngleAsRadians() + pitchOffsetRadians).withLength(xy.magnitude()); // Turn
        var rot = -joystick.getTwist();

        if (RobotContainer.controls.controlMode == Controls.ControlMode.SEPARATE_ACCELERATION) {
            xy = xy.normalized().scaled(joystick.getZ());
        }

        // Brake if input is 0
        if (xy.magnitude() < RobotContainer.controls.deadBandDrive
                && abs(rot) < RobotContainer.controls.deadBandDrive) {
            RobotContainer.drive.stopAllMotors();
            return;
        }

        // Apply deadband
        xy = applyDeadband(xy, RobotContainer.controls.deadBandDrive)
                .scaled(RobotContainer.controls.getAccelerationSensitivity());
        rot = applyDeadband(rot, RobotContainer.controls.deadBandTurn)
                * RobotContainer.controls.turnSensitivity;

        // Apply slew rate
        if (RobotContainer.controls.slewRateLimited) {
            var xLimited = xLimiter.calculate(abs(xy.x)) * signum(xy.x);
            var yLimited = yLimiter.calculate(abs(xy.y)) * signum(xy.y);
            // rot = rotLimiter.calculate(abs(rot)) * signum(rot);
            if (Double.isNaN(xLimited) || Double.isNaN(yLimited)) {
                xLimiter.reset(xy.x);
                yLimiter.reset(xy.y);
            } else {
                xy = new Vector2(xLimited, yLimited);
            }
        }

        // Convert to velocity
        xy.x = RobotContainer.drive.percent2driveVelocity(xy.x);
        xy.y = RobotContainer.drive.percent2driveVelocity(xy.y);
        rot = RobotContainer.drive.percent2rotationVelocity(rot);

        setChassisSpeeds(xy, rot);
        // System.out.println(xy.toString() + rot);
    }

    public static Vector2 applyDeadband(Vector2 xy, double deadBand) {
        return xy.normalized().scaled(applyDeadband(xy.magnitude(), deadBand));
    }

    public static double applyDeadband(double x, double deadBand) {
        return abs(x) < deadBand ? 0 : (abs(x) - deadBand) / (1 - deadBand) * signum(x);
    }

    private void setChassisSpeeds(Vector2 vxy, double vRot) {
        switch (RobotContainer.controls.driveOrientation) {
            case Forwards:
                RobotContainer.drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(vxy.x, vxy.y,
                        vRot, new Rotation2d(0.0)));
                break;
            case Backwards:
                RobotContainer.drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(vxy.x, vxy.y,
                        vRot, Rotation2d.fromRadians(Math.PI)));
                break;
            case FieldOriented:
                RobotContainer.drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(vxy.x, vxy.y,
                        vRot, Rotation2d.fromDegrees(FridoNavx.getInstance().getAngle())));
                break;
        }
    }
}

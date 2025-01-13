package frc.robot.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.fridowpi.utils.Vector2;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Controls;

public class DriveCommand extends Command {

    public DriveCommand(SwerveDrive drive) {
        addRequirements(drive);
    }

    public void execute() {

        var pitchOffsetRadians = Radians.convertFrom(90, Degrees);

        var joystick = RobotContainer.controls.driveJoystick;
        var xy = new Vector2(joystick.getX(), joystick.getY());
        xy = Vector2.fromRadians(xy.getAngleAsRadians() + pitchOffsetRadians).withLength(xy.magnitude()); // Turn
        var rot = -joystick.getTwist();

        if (RobotContainer.controls.controlMode == Controls.ControlMode.SEPARATE_ACCELERATION) {
            xy = xy.normalized().scaled(joystick.getZ());
        }

        // Brake if input is 0
        /*
         * if (xy.magnitude() < Controls.deadBandDrive
         * && Math.abs(rot) < Controls.deadBandTurn) {
         * drive.stopAllMotors();
         * return;
         * }
         */

        // Apply deadband
        xy = applyDeadband(xy, Controls.deadBandDrive)
                .scaled(RobotContainer.controls.getAccelerationSensitivity());
        rot = applyDeadband(rot, Controls.deadBandTurn)
                * RobotContainer.controls.turnSensitivity;

        // Apply slew rate
        /*
         * if (Controls.isSlewRateLimited()) {
         * var xLimited = xLimiter.calculate(abs(xy.x)) * signum(xy.x);
         * var yLimited = yLimiter.calculate(abs(xy.y)) * signum(xy.y);
         * // rot = rotLimiter.calculate(abs(rot)) * signum(rot);
         * if (Double.isNaN(xLimited) || Double.isNaN(yLimited)) {
         * xLimiter.reset(xy.x);
         * yLimiter.reset(xy.y);
         * } else {
         * xy = new Vector2(xLimited, yLimited);
         * }
         * }
         */

        // Convert to velocity
        xy.scale(Constants.SwerveDrive.maxSpeed);
        rot *= Constants.SwerveDrive.maxTurnSpeed;

        setChassisSpeeds(xy, rot);
    }

    public static Vector2 applyDeadband(Vector2 xy, double deadBand) {
        return xy.normalized().scaled(applyDeadband(xy.magnitude(), deadBand));
    }

    public static double applyDeadband(double x, double deadBand) {
        return Math.abs(x) < deadBand ? 0 : (Math.abs(x) - deadBand) / (1 - deadBand) * Math.signum(x);
    }

    private void setChassisSpeeds(Vector2 vxy, double vRot) {
        switch (RobotContainer.controls.driveOrientation) {
            case Forwards:
                RobotContainer.drive.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vxy.x, vxy.y,
                        vRot, new Rotation2d(0.0)));
                break;
            case Backwards:
                RobotContainer.drive.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vxy.x, vxy.y,
                        vRot, Rotation2d.fromRadians(Math.PI)));
                break;
            case FieldOriented:
                RobotContainer.drive.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vxy.x, vxy.y,
                        vRot, Rotation2d.fromDegrees(RobotContainer.gyro.getAngle())));
                break;
        }
    }
}

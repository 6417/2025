import org.junit.jupiter.api.Test;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.fridowpi.utils.AccelerationLimiter;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Random;

class AccelerationLimiterTest {
    double maxAccel = 100.0; // m/s^2
    double radius = 0.267;

    double maxVel = 4.5;
    final double maxOmega = 2.0; // 4.5 * 2 * Math.PI / (Math.sqrt(2) * radius);

    @Test
    void randomTestWillTerminate() {
        AccelerationLimiter limiter = new AccelerationLimiter(maxAccel, radius);
        Random rng = new Random(10230);

        for (int i = 0; i < 1000; i++) {
            double vx = rng.nextDouble(-maxVel, maxVel);
            double vy = rng.nextDouble(-maxVel, maxVel);
            double omega = rng.nextDouble(-maxOmega, maxOmega);

            double currVx = rng.nextDouble(-maxVel, maxVel);
            double currVy = rng.nextDouble(-maxVel, maxVel);
            double currOmega = rng.nextDouble(-maxOmega, maxOmega);

            ChassisSpeeds current = new ChassisSpeeds(currVx, currVy, currOmega);
            ChassisSpeeds desired = new ChassisSpeeds(vx, vy, omega);

            double dt = rng.nextDouble(0.0001, 0.01);

            ChassisSpeeds res = limiter.constrain(current, desired, dt);

            assertTrue(Math.abs(res.vxMetersPerSecond) < maxVel * 1.01);
            assertTrue(Math.abs(res.vyMetersPerSecond) < maxVel * 1.01);
            assertTrue(Math.abs(res.omegaRadiansPerSecond) < maxOmega * 1.01);
        }
    }
}

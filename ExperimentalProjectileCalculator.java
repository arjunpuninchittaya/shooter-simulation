package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utils.math.Vector;

/**
 * Experimental projectile calculator implementing the projectile math from the provided PDF
 * and the screenshot. It computes hood angle and flywheel RPM (with an optional velocity
 * compensation pass for robot motion) and returns a ShotSolution containing the result.
 *
 * Units:
 * - Distances: inches
 * - Gravity: inches / sec^2
 * - Linear speeds: inches / sec
 * - Output RPM is revolutions per minute of the flywheel
 */
@Config
public class ExperimentalProjectileCalculator {

    // Gravity (ft/s^2 * 12 = inches/s^2)
    public static double G = 32.174 * 12.0;

    // Hood limits (radians)
    public static double HOOD_MIN_ANGLE = Math.toRadians(90-ShooterConverter.getAngleFromHood(0.95));
    public static double HOOD_MAX_ANGLE = Math.toRadians(90-ShooterConverter.getAngleFromHood(0.27));

    // Flywheel physical radius in inches (change to match your hardware)
    public static double FLYWHEEL_RADIUS_INCH = 1.4173;

    // Safety epsilon to avoid divide-by-zero
    private static final double EPS = 1e-8;

    public static class ShotSolution {
        public final double flywheelRPM;   // RPM to set the flywheel to
        public final double hoodAngle;     // servo / hood angle (radians or normalized depending on your hood mapping)
        public final double turretVelocityCompOffset; // turret angular offset (radians) due to perpendicular robot velocity
        public final boolean valid;

        public ShotSolution(double flywheelRPM, double hoodAngle, double turretVelocityCompOffset, boolean valid) {
            this.flywheelRPM = flywheelRPM;
            this.hoodAngle = hoodAngle;
            this.turretVelocityCompOffset = turretVelocityCompOffset;
            this.valid = valid;
        }
    }

    private static double clamp(double v, double max, double min) {
        if (Double.isNaN(v)) return Double.NaN;
        return Math.max(min, Math.min(max, v));
    }


    /**
     * Compute a shot solution.
     *
     * @param x horizontal distance to target (inches)
     * @param y vertical difference target height - shooter height (inches)
     * @param a an initial reference angle (radians) - in the screenshot they use a 'score angle' from constants
     * @param robotVelocity robot velocity vector (inches/sec) in a robot-centered frame (magnitude + heading)
     * @param robotToGoalTheta angle from robot to goal (radians) - used to compute components of robot velocity relative to shot direction
     * @param robotHeading robot heading (radians) - used for turret angle compensation (if needed)
     * @return ShotSolution with RPM and hoodAngle; valid=false if math failed
     */
    public static ShotSolution calculateShot(double x, double y, double a, Vector robotVelocity, double robotToGoalTheta, double robotHeading) {
        // Basic checks
        if (x <= EPS) return new ShotSolution(Double.NaN, Double.NaN, 0, false);

        // Step 1: Calculate initial launch components
        double hoodAngle = Double.NaN;
        try {
            hoodAngle = Math.atan(2.0 * y / x - Math.tan(a));
            hoodAngle = clamp(hoodAngle, HOOD_MAX_ANGLE, HOOD_MIN_ANGLE);
        } catch (Exception ex) {
            return new ShotSolution(Double.NaN, Double.NaN, 0, false);
        }

        // Initial launch speed (linear) using formula:
        // v = sqrt( g * x^2 / (2 * cos(θ)^2 * (x tan θ - y)) )
        double denom = 2.0 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y);
        if (denom <= EPS) return new ShotSolution(Double.NaN, hoodAngle, 0, false);
        double flywheelSpeed = Math.sqrt(G * x * x / denom);

        // Step 2: Decompose robot velocity into parallel and perpendicular components
        // relative to the robot-to-goal vector
        double parallelComponent = 0.0;
        double perpendicularComponent = 0.0;
        
        if (robotVelocity != null && robotVelocity.getMagnitude() > EPS) {
            double robotVelMag = robotVelocity.getMagnitude();
            
            // Angle between robot velocity and shot direction
            double coordinateTheta = robotVelocity.getTheta() - robotToGoalTheta;
            
            // Parallel component (negative = moving away from goal)
            parallelComponent = -Math.cos(coordinateTheta) * robotVelMag;
            
            // Perpendicular component
            perpendicularComponent = Math.sin(coordinateTheta) * robotVelMag;
        }

        // Step 3: Calculate velocity compensation
        double vz = flywheelSpeed * Math.sin(hoodAngle); // vertical component of launch velocity
        double time = flywheelSpeed * Math.cos(hoodAngle) > EPS ? x / (flywheelSpeed * Math.cos(hoodAngle)) : Double.NaN;
        if (Double.isNaN(time) || time <= 0.0) return new ShotSolution(Double.NaN, hoodAngle, 0, false);

        // Ideal velocity requirement to hit the adjusted target
        double ivr = x / time + parallelComponent;
        
        // New velocity requirement accounting for perpendicular motion
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        
        // New distance requirement
        double ndr = nvr * time;

        // Step 4: Recalculate launch components with compensated values
        double hoodAngle2 = Math.atan(vz / (nvr + EPS));
        hoodAngle2 = clamp(hoodAngle2, HOOD_MAX_ANGLE, HOOD_MIN_ANGLE);

        // Recalculate linear speed using the adjusted distance ndr and new hoodAngle2
        double denom2 = 2.0 * Math.pow(Math.cos(hoodAngle2), 2) * (ndr * Math.tan(hoodAngle2) - y);
        if (denom2 <= EPS) return new ShotSolution(Double.NaN, hoodAngle2, 0, false);
        double flywheelSpeed2 = Math.sqrt(G * ndr * ndr / denom2);

        // Step 5: Calculate turret yaw offset for perpendicular velocity compensation
        double turretVelCompOffset = 0.0;
        if (Math.abs(ivr) > EPS) {
            turretVelCompOffset = Math.atan(perpendicularComponent / ivr);
        } else {
            // if ivr ~ 0, offset is ±pi/2 depending on perpendicularComponent sign
            if (Math.abs(perpendicularComponent) > EPS) {
                turretVelCompOffset = Math.signum(perpendicularComponent) * Math.PI / 2.0;
            } else {
                turretVelCompOffset = 0.0;
            }
        }

        // Verify solution is valid
        boolean valid = !(Double.isNaN(flywheelSpeed2) || Double.isNaN(hoodAngle2));

        // Return shot solution with final hood angle and flywheel speed
        return new ShotSolution(flywheelSpeed2, hoodAngle2, turretVelCompOffset, valid);
    }
}
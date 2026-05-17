package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.NavigableMap;
import java.util.TreeMap;

/**
 * ShooterSubsystem: A sophisticated class that manages the robot's motorized launcher.
 *
 * Key Features:
 * 1. Automatic Turret: Rotates to keep the launcher facing the goal regardless of driving.
 * 2. Flywheel Control: Two high-speed motors with PIDF velocity regulation.
 * 3. Adjustable Hood: Changes launch angle based on the robot's distance from the goal.
 * 4. Prediction: Adjusts aiming to hit the goal even while the robot is driving at speed.
 */
public class ShooterSubsystem {

    // --- Physical Hardware Components ---
    private final Servo     turretServo1; // The primary servo that rotates the turret base.
    private final Servo     turretServo2; // The secondary servo working in sync with the first.
    private final Servo     hoodServo2;   // The servo that tilts the hood to adjust shot trajectory.
    private final DcMotorEx flywheel1;    // The first high-speed launcher motor.
    private final DcMotorEx flywheel2;    // The second high-speed launcher motor.

    // --- Ballistics and Physics Constants ---
    private final double PROJECTILE_SPEED_IPS = 152;  // The estimated speed of the ball (152 inches per second).
    private final int    ENCODER_CPR          = 28;   // The number of encoder counts in one full motor revolution.

    // --- Turret Calibration ---
    private final double TURRET_RANGE_DEG     = 360.0; // The total physical degrees the turret is capable of spinning.
    private final double TURRET_CENTER_POS    = 0.805; // The servo position (0 to 1) that represents physical "Forward".

    // --- Motor Speed Control (PIDF) ---
    // P (Proportional) helps the motor reach speed quickly; F (Feedforward) maintains speed based on voltage.
    private static final double P = 40.0;
    private static final double F = 11.96;

    // --- Real-time Compensation ---
    // Adjusts the hood and RPM slightly when the motors slow down during a shot.
    private final double HOOD_RPM_COMPENSATION_SCALE = 0.00045;
    private final double RPM_BOOST_SCALE = 1.35;

    private double currentTurretAngle = 0.0; // Tracks the current turret angle in degrees.

    // --- Lookup Tables (Tuning Data) ---
    // These maps store "Distance to Goal" vs. "Required Servo/RPM" values determined through testing.
    private final NavigableMap<Double, Double> hoodLookup = new TreeMap<>();
    private final NavigableMap<Double, Double> rpmLookup  = new TreeMap<>();

    /**
     * Constructor: Initializes the shooter hardware and loads the tuning tables.
     */
    public ShooterSubsystem(HardwareMap hardwareMap) {
        // Map the servos and hood from the hardware configuration.
        turretServo1 = hardwareMap.get(Servo.class, "turret1");
        turretServo2 = hardwareMap.get(Servo.class, "turret2");
        hoodServo2 = hardwareMap.get(Servo.class, "hood2");

        // Map and configure the flywheel motors.
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        // Ensure motors spin in opposite directions to pinch and launch the ball.
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Tell the motors to use their built-in encoders for precise speed control.
        flywheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Apply our custom PIDF values to make the flywheels spin at a very consistent speed.
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheel1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheel2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // --- Hood Tuning Table: (Distance in inches, Servo Position) ---
        hoodLookup.put(40.0,  0.00);
        hoodLookup.put(60.0,  0.15);
        hoodLookup.put(90.0,  0.25);
        hoodLookup.put(108.0, 0.25);
        hoodLookup.put(120.0, 0.23);
        hoodLookup.put(133.0, 0.4);
        hoodLookup.put(150.0, 0.4);
        hoodLookup.put(163.0, 0.4);

        // --- RPM Tuning Table: (Distance in inches, Motor RPM) ---
        rpmLookup.put(40.0,  2700.0);
        rpmLookup.put(60.0,  2900.0);
        rpmLookup.put(90.0,  3400.0);
        rpmLookup.put(108.0, 3500.0);
        rpmLookup.put(120.0, 3800.0);
        rpmLookup.put(133.0, 4100.0);
        rpmLookup.put(150.0, 4400.0);
        rpmLookup.put(163.0, 4600.0);

        // Initialize the turret to face physical forward.
        setTurretAngle(0.0);
    }

    /**
     * update(): The core logic loop. This calculates aiming and power every few milliseconds.
     *
     * @param pose Current field position of the robot.
     * @param velocity Current movement speed of the robot.
     * @param goalX The field X coordinate of the target goal.
     * @param goalY The field Y coordinate of the target goal.
     */
    public void update(Pose pose, Vector velocity, double goalX, double goalY, Telemetry telemetry) {

        // 1. Calculate current distance to the target.
        double dx       = goalX - pose.getX();
        double dy       = goalY - pose.getY();
        double distance = Math.hypot(dx, dy);

        // 2. Predict Future Position: If we are driving, the ball needs to be aimed where the goal WILL be
        // relative to us by the time the ball finishes its flight.
        double flightTime = distance / PROJECTILE_SPEED_IPS; // Time in seconds for the ball to travel.
        double futureX    = pose.getX() + velocity.getXComponent() * flightTime;
        double futureY    = pose.getY() + velocity.getYComponent() * flightTime;

        // 3. Aim Turret: Calculate the angle to the goal, then subtract the robot's current heading.
        dx = goalX - futureX;
        dy = goalY - futureY;
        double localizationAngle = Math.toDegrees(Math.atan2(dy, dx))
                - Math.toDegrees(pose.getHeading());
        localizationAngle = wrapAngle(localizationAngle); // Ensure angle is within -180 to +180.
        setTurretAngle(localizationAngle); // Rotate the servos.

        // 4. Calculate Power & Trajectory: Look up the RPM and Hood Angle for the *predicted* future distance.
        double futureDx      = goalX - futureX;
        double futureDy      = goalY - futureY;
        double futureDistance = Math.hypot(futureDx, futureDy);

        double targetRPM     = getRPM(futureDistance);
        double hoodPos       = Math.max(0.0, Math.min(0.4, getHoodAngle(futureDistance)));

        // 5. Shot Recovery: If the motors slow down because a ball just went through,
        // temporarily boost the target RPM and adjust the hood to stay accurate.
        double currentRPM    = (flywheel1.getVelocity()) * 60.0 / ENCODER_CPR; // Calculate RPM from motor velocity.
        double rpmDrop       = Math.max(0.0, targetRPM - currentRPM); // Measure how much we've slowed down.
        targetRPM         = targetRPM + (rpmDrop * RPM_BOOST_SCALE); // Apply boost.
        double compensation  = rpmDrop * HOOD_RPM_COMPENSATION_SCALE;
        hoodPos = Math.max(0.0, Math.min(0.4, hoodPos - compensation)); // Adjust hood.

        // 6. Execute: Command the motors and servos to their calculated states.
        hoodServo2.setPosition(hoodPos);
        double velocityTPS = targetRPM * ENCODER_CPR / 60.0; // Convert RPM back to Ticks Per Second for the motor.
        flywheel1.setVelocity(velocityTPS);
        flywheel2.setVelocity(velocityTPS);

        // Debugging output for the driver station.
        telemetry.addData("[Shooter] Distance",        "%.1f in", distance);
        telemetry.addData("[Shooter] Hood Position",   "%.4f",    hoodPos);
        telemetry.addData("[Shooter] Hood Compensation","%.4f",   compensation);
        telemetry.addData("[Shooter] Future Distance",   "%.4f",   futureDistance);
        telemetry.addData("[Shooter] RPM Drop",        "%.0f",    rpmDrop);
        telemetry.addData("[Shooter] Target RPM",      "%.0f",    targetRPM);
        telemetry.addData("[Shooter] Current RPM",     "%.0f",    currentRPM);
        telemetry.addData("[Shooter] get hood angle     "," %.4f",    getHoodAngle(futureDistance));
        telemetry.addData("[Shooter] Turret Cmd",      "%.2f°",   currentTurretAngle);
    }

    /**
     * isAtTargetRPM(): Returns true if the flywheels are spinning fast enough to shoot accurately.
     */
    public boolean isAtTargetRPM(double distance) {
        double target    = getRPM(distance) * ENCODER_CPR / 60.0;
        double tolerance = target * 0.05; // We accept a 5% speed difference.
        return Math.abs(flywheel1.getVelocity() - target) < tolerance
                && Math.abs(flywheel2.getVelocity() - target) < tolerance;
    }

    /**
     * getTurretAngle(): Returns the current software angle of the turret.
     */
    public double getTurretAngle() { return currentTurretAngle; }

    /**
     * addTelemetry(): Adds detailed motor speed and angle data to the dashboard.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("[Shooter] Turret Angle",  "%.1f°", currentTurretAngle);
        telemetry.addData("[Shooter] Flywheel1 RPM", "%.0f",  flywheel1.getVelocity() * 60.0 / ENCODER_CPR);
        telemetry.addData("[Shooter] Flywheel2 RPM", "%.0f",  flywheel2.getVelocity() * 60.0 / ENCODER_CPR);
    }

    /**
     * Internal helper to convert a degree angle into a servo position and apply it.
     */
    private void setTurretAngle(double angleDeg) {
        // Safety Clamp: Prevent the turret from rotating so far that it breaks its own wires.
        angleDeg = Math.max(-295.0, Math.min(60, angleDeg));
        currentTurretAngle = angleDeg;

        // Map the degree value to the 0.0 - 1.0 range used by servos.
        double servoPos = angleDeg / TURRET_RANGE_DEG + TURRET_CENTER_POS;
        servoPos = Math.max(0.0, Math.min(1.0, servoPos)); // Final protection clamp.

        turretServo1.setPosition(servoPos);
        turretServo2.setPosition(servoPos);
    }

    /** Helper to ensure angles wrap correctly (e.g., 190 becomes -170) */
    private double wrapAngle(double angle) {
        while (angle >  180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    // Lookup functions: they find the best value from the tuning tables for a given distance.
    private double getHoodAngle(double distance) { return interpolate(hoodLookup, distance); }
    private double getRPM(double distance)       { return interpolate(rpmLookup,  distance); }

    /**
     * Linear Interpolation: If the robot is at a distance not exactly in our table,
     * this math calculates a weighted average between the two closest points.
     */
    private double interpolate(NavigableMap<Double, Double> table, double key) {
        if (table.containsKey(key)) return table.get(key); // Perfect match found.
        Double lower = table.floorKey(key);   // Find the next lowest distance in table.
        Double upper = table.ceilingKey(key); // Find the next highest distance in table.

        if (lower == null) return table.get(table.firstKey()); // Distance is smaller than our table.
        if (upper == null) return table.get(table.lastKey());  // Distance is larger than our table.

        double vLower = table.get(lower);
        double vUpper = table.get(upper);

        // The standard interpolation formula: y = y1 + (y2 - y1) * ((x - x1) / (x2 - x1))
        return vLower + (vUpper - vLower) * (key - lower) / (upper - lower);
    }
}

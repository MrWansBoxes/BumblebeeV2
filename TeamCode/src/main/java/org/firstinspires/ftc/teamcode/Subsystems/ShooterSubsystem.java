package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.NavigableMap;
import java.util.TreeMap;

public class ShooterSubsystem {

    // Hardware
    private final Servo     turretServo1;
    private final Servo     turretServo2;
    private final Servo     hoodServo2;
    private final DcMotorEx flywheel1;
    private final DcMotorEx flywheel2;

    // Variables
    private final double PROJECTILE_SPEED_IPS = 152;  // inches per second that the artifact travels at, used for shoot while moving 152
    private final int    ENCODER_CPR          = 28;   // encoder counts per revolution

    // Turret
    private final double TURRET_RANGE_DEG     = 360.0; // degrees of turret range
    private final double TURRET_CENTER_POS    = 0.825; // servo position for physical forward

    // PIDF for flywheel velocity control
    private static final double P = 40.0;
    private static final double F = 11.96;

    // Hood angle and RPM compensation scale based on RPM drop
    private final double HOOD_RPM_COMPENSATION_SCALE = 0.00045;
    private final double RPM_BOOST_SCALE = 1.35;


    private double currentTurretAngle = 0.0;


    private final NavigableMap<Double, Double> hoodLookup = new TreeMap<>();
    private final NavigableMap<Double, Double> rpmLookup  = new TreeMap<>();


    // Constructor
    public ShooterSubsystem(HardwareMap hardwareMap) {

        // Turret
        turretServo1 = hardwareMap.get(Servo.class, "turret1");
        turretServo2 = hardwareMap.get(Servo.class, "turret2");

        // Hood
        hoodServo2 = hardwareMap.get(Servo.class, "hood2");

        // Flywheels
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set PIDF for flywheel
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheel1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheel2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Hood lookup table (Distance in inches, Hood position)
        hoodLookup.put(40.0,  0.00);
        hoodLookup.put(60.0,  0.15);
        hoodLookup.put(90.0,  0.25);
        hoodLookup.put(108.0, 0.25);
        hoodLookup.put(120.0, 0.23);
        hoodLookup.put(133.0, 0.4);
        hoodLookup.put(150.0, 0.4);
        hoodLookup.put(163.0, 0.4);

        // RPM lookup table (Distance in inches, RPM)
        rpmLookup.put(40.0,  2700.0);
        rpmLookup.put(60.0,  2900.0);
        rpmLookup.put(90.0,  3400.0);
        rpmLookup.put(108.0, 3500.0);
        rpmLookup.put(120.0, 3800.0);
        rpmLookup.put(133.0, 4100.0);
        rpmLookup.put(150.0, 4400.0);
        rpmLookup.put(163.0, 4600.0);

        // Home turret to forward
        setTurretAngle(0.0);
    }

    // Main method
    public void update(Pose pose, Vector velocity, double goalX, double goalY, Telemetry telemetry) {

        // Distance to goal
        double dx       = goalX - pose.getX();
        double dy       = goalY - pose.getY();
        double distance = Math.hypot(dx, dy);

        // Lead target for moving shots
        double flightTime = distance / PROJECTILE_SPEED_IPS;
        double futureX    = pose.getX() + velocity.getXComponent() * flightTime;
        double futureY    = pose.getY() + velocity.getYComponent() * flightTime;

        // Turret angle from localization
        dx = goalX - futureX;
        dy = goalY - futureY;
        double localizationAngle = Math.toDegrees(Math.atan2(dy, dx))
                - Math.toDegrees(pose.getHeading());
        localizationAngle = wrapAngle(localizationAngle);
        setTurretAngle(localizationAngle);

        // Hood angle from future distance
        double futureDx       = goalX - futureX;
        double futureDy       = goalY - futureY;
        double futureDistance = Math.hypot(futureDx, futureDy);
        double targetRPM      = getRPM(futureDistance);
        double hoodPos        = Math.max(0.0, Math.min(0.4, getHoodAngle(futureDistance)));
        double currentRPM     = (flywheel1.getVelocity()) * 60.0 / ENCODER_CPR;
        double rpmDrop        = Math.max(0.0, targetRPM - currentRPM);
        double commandedRPM   = targetRPM + (rpmDrop * RPM_BOOST_SCALE);
        double compensation   = rpmDrop * HOOD_RPM_COMPENSATION_SCALE;
        hoodPos = Math.max(0.0, Math.min(0.4, hoodPos - compensation));
        hoodServo2.setPosition(hoodPos);

        // Flywheel speed from future distance
        double velocityTPS = commandedRPM * ENCODER_CPR / 60.0;
        flywheel1.setVelocity(velocityTPS);
        flywheel2.setVelocity(velocityTPS);

        // Telemetry
        telemetry.addData("[Shooter] Distance",        "%.1f in", distance);
        telemetry.addData("[Shooter] Future Distance",  "%.1f in", futureDistance);
        telemetry.addData("[Shooter] Target RPM",      "%.0f",    targetRPM);
        telemetry.addData("[Shooter] Commanded RPM",   "%.0f",    commandedRPM);
        telemetry.addData("[Shooter] Current RPM",     "%.0f",    currentRPM);
        telemetry.addData("[Shooter] RPM Drop",        "%.0f",    rpmDrop);
        telemetry.addData("[Shooter] Hood Position",   "%.3f",    hoodPos);
        telemetry.addData("[Shooter] Hood Comp",       "%.4f",    compensation);
        telemetry.addData("[Shooter] Turret Cmd",      "%.2f°",   currentTurretAngle);
        telemetry.addData("[Shooter] Field Angle",     "%.2f°",   Math.toDegrees(Math.atan2(dy, dx)));
        telemetry.addData("[Shooter] Heading",         "%.2f°",   Math.toDegrees(pose.getHeading()));
        telemetry.update();
    }


    // Public methods

    /** True when both flywheels are within 5% of target RPM for current distance */
    public boolean isAtTargetRPM(double distance) {
        double target    = getRPM(distance) * ENCODER_CPR / 60.0;
        double tolerance = target * 0.05;
        return Math.abs(flywheel1.getVelocity() - target) < tolerance
                && Math.abs(flywheel2.getVelocity() - target) < tolerance;
    }

    public double getTurretAngle() { return currentTurretAngle; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("[Shooter] Turret Angle",  "%.1f°", currentTurretAngle);
        telemetry.addData("[Shooter] Flywheel1 RPM", "%.0f",  flywheel1.getVelocity() * 60.0 / ENCODER_CPR);
        telemetry.addData("[Shooter] Flywheel2 RPM", "%.0f",  flywheel2.getVelocity() * 60.0 / ENCODER_CPR);
    }

    // Private helpers

    private void setTurretAngle(double angleDeg) {
        // Clamp angle to wire limits
        angleDeg = Math.max(-295.0, Math.min(60, angleDeg));
        currentTurretAngle = angleDeg;
        double servoPos = angleDeg / TURRET_RANGE_DEG + TURRET_CENTER_POS;
        // Clamp prevents Axons from resetting its reference position
        servoPos = Math.max(0.0, Math.min(1.0, servoPos));
        turretServo1.setPosition(servoPos);
        turretServo2.setPosition(servoPos);
    }

    private double wrapAngle(double angle) {
        while (angle >  180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private double getHoodAngle(double distance) { return interpolate(hoodLookup, distance); }
    private double getRPM(double distance)       { return interpolate(rpmLookup,  distance); }

    private double interpolate(NavigableMap<Double, Double> table, double key) {
        if (table.containsKey(key)) return table.get(key);
        Double lower = table.floorKey(key);
        Double upper = table.ceilingKey(key);
        if (lower == null) return table.get(table.firstKey());
        if (upper == null) return table.get(table.lastKey());
        double vLower = table.get(lower);
        double vUpper = table.get(upper);
        return vLower + (vUpper - vLower) * (key - lower) / (upper - lower);
    }
}
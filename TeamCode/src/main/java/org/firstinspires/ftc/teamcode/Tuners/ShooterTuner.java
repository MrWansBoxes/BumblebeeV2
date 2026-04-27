package org.firstinspires.ftc.teamcode.Tuners;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

@Configurable
@TeleOp
public class ShooterTuner extends OpMode {

    // -------------------------------------------------------------------------
    // Adjust these live from the Panels dashboard
    // -------------------------------------------------------------------------
    public static double hoodPosition  = 0;  // raw servo position 0.0–0.4

    public static double turretAngle   = 0;  // degrees, -170 to +170

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------

    private IntakeSubsystem intake;
    private Servo    turretServo1;
    private Servo    turretServo2;
    private Servo    hoodServo2;
    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;
    private DcMotor intakeMotor1;
    private DcMotor intakeMotor2;

    private TelemetryManager telemetryM;

    // Match these to ShooterSubsystem constants
    private static final double TURRET_SOFT_LIMIT = 170;
    private static final double TURRET_RANGE_DEG  = 340;
    public static double TARGET_RPM               = 4500;
    private static final int    ENCODER_CPR       = 28;
    private static double P                 = 40;
    private static double F                 = 11.96;
    private static double turretPosition = 1;


    // -------------------------------------------------------------------------
    // Init
    // -------------------------------------------------------------------------
    @Override
    public void init() {
        turretServo1 = hardwareMap.get(Servo.class, "turret1");
        turretServo2 = hardwareMap.get(Servo.class, "turret2");
        hoodServo2   = hardwareMap.get(Servo.class, "hood2");
        flywheel1    = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2    = hardwareMap.get(DcMotorEx.class, "flywheel2");
        intakeMotor1 = hardwareMap.get(DcMotor.class, "intake1");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "intake2");

        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0.0, 0.0, F);
        flywheel1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheel2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        turretServo1.setPosition(1);
        turretServo2.setPosition(1);
       // setTurret(0);
        hoodServo2.setPosition(0);
    }

    // -------------------------------------------------------------------------
    // Loop
    // -------------------------------------------------------------------------
    @Override
    public void loop() {



        // Clamp inputs to safe ranges
        double clampedHood   = Math.max(0.0, Math.min(0.4, hoodPosition));
        double clampedTurret = Math.max(-TURRET_SOFT_LIMIT, Math.min(TURRET_SOFT_LIMIT, turretAngle));

        // Command hood — servo2 mirrored
        hoodServo2.setPosition(clampedHood);

        if (gamepad1.aWasPressed()){
            flywheel1.setVelocity(0);
            flywheel2.setVelocity(0);
        }
        // Command turret
    //    setTurret(clampedTurret);
        turretServo1.setPosition(turretPosition);
        turretServo2.setPosition(turretPosition);
        // Flywheels always spinning so you can fire test shots
        double velocityTPS = TARGET_RPM * ENCODER_CPR / 60.0;
        
        if (gamepad1.bWasPressed()) {
            flywheel1.setVelocity(velocityTPS);
            flywheel2.setVelocity(velocityTPS);
        }
        if (gamepad1.xWasPressed()){
            intakeMotor1.setPower(-1);
            intakeMotor2.setPower(1);
        }
        if (gamepad1.yWasPressed()){
            intakeMotor1.setPower(0);
            intakeMotor2.setPower(0);
        }
        // Telemetry
        double fw1RPM = flywheel1.getVelocity() * 60.0 / ENCODER_CPR;
        double fw2RPM = flywheel2.getVelocity() * 60.0 / ENCODER_CPR;
        boolean atRPM = Math.abs(fw1RPM - TARGET_RPM) < TARGET_RPM * 0.05
                && Math.abs(fw2RPM - TARGET_RPM) < TARGET_RPM * 0.05;
        double error = velocityTPS - fw1RPM;

        telemetryM.debug("error", error);
        telemetryM.debug("-- TUNING VALUES --");
        telemetryM.debug("Hood position",        clampedHood);
        telemetryM.debug("Hood servo2 pos",      clampedHood);
        telemetryM.debug("Turret angle (deg)",   clampedTurret);
        telemetryM.debug("Turret servo pos",     (clampedTurret + TURRET_SOFT_LIMIT) / TURRET_RANGE_DEG);
        telemetryM.debug("-- FLYWHEEL --");
        telemetryM.debug("Flywheel1 RPM",        fw1RPM);
        telemetryM.debug("Flywheel2 RPM",        fw2RPM);
        telemetryM.debug("Flywheel1 TPS",        flywheel1.getVelocity());
        telemetryM.debug("Flywheel2 TPS",        flywheel2.getVelocity());
        telemetryM.debug("At Target RPM",        atRPM);
        telemetryM.update(telemetry);
    }

    // -------------------------------------------------------------------------
    // Helper
    // -------------------------------------------------------------------------
    private void setTurret(double angleDeg) {
        double servoPos = (angleDeg + TURRET_SOFT_LIMIT) / TURRET_RANGE_DEG;
        servoPos = Math.max(0.0, Math.min(1.0, servoPos));
        turretServo1.setPosition(servoPos);
        turretServo2.setPosition(servoPos);
    }
}
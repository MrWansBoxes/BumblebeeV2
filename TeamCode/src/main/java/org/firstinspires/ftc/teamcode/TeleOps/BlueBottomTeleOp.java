
package org.firstinspires.ftc.teamcode.TeleOps;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class BlueBottomTeleOp extends OpMode {

    private Follower follower;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private TelemetryManager telemetryM;

    // blue side goal position
    private static final double GOAL_X = 7.385694249649367;
    private static final double GOAL_Y = 135.31697054698458;

    private int intakeFlag = 0;
    private int gateFlag = 0;



    // auto sets this from the auto opmode, falls back to default if run standalone
    private static final Pose DEFAULT_STARTING_POSE = new Pose(35.039, 9.183, Math.toRadians(180));
    public static Pose startingPose = null;

    private boolean automatedDrive = false;
    private Supplier<PathChain> pathChain;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose != null ? startingPose : DEFAULT_STARTING_POSE);
        follower.update();

        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        intake.start();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        // driver sticks, unless an automated path is running
        if (!automatedDrive) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // robot centric
            );
        }

        // shooter tracks the goal every loop
        Pose pose = follower.getPose();
        Vector velocity = follower.getVelocity();
        shooter.update(pose, velocity, GOAL_X, GOAL_Y, telemetry);

        // A toggles intake on/off, B toggles reverse
        if (gamepad1.aWasPressed()) {
            if (intakeFlag == 0) {
                intake.intakeOn();
                intakeFlag = 1;
            } else if (intakeFlag == 1) {
                intake.intakeOff();
                intakeFlag = 0;
            } else if (intakeFlag == -1) {
                intake.intakeOff();
                intakeFlag = 0;
            }
        }

        if (gamepad1.bWasPressed()) {
            if (intakeFlag == 0) {
                intake.intakeReverse();
                intakeFlag = -1;
            } else if (intakeFlag == -1) {
                intake.intakeOff();
                intakeFlag = 0;
            } else if (intakeFlag == 1) {
                intake.intakeOff();
                intakeFlag = 0;
            }
        }


        // Y toggles the gate
        if (gamepad1.yWasPressed()) {
            if (gateFlag == 0) {
                intake.gateOpen();
                gateFlag = 1;
            } else if (gateFlag == 1) {
                intake.gateClose();
                gateFlag = 0;
            }
        }
        if (gamepad2.xWasPressed()) {
            follower.setX(133.1998597475456);
            follower.setY(30.822580645161285);
            follower.setHeading(Math.toRadians(180));
        }
        // dpad up starts automated path, dpad down or path finishing cancels it
        if (gamepad1.dpadUpWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        if (automatedDrive && (gamepad1.dpadDownWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        shooter.addTelemetry(telemetry);
        //  telemetryM.debug("position",      follower.getPose());
        //  telemetryM.debug("velocity",      follower.getVelocity());
        //  telemetryM.debug("automatedDrive", automatedDrive);
        telemetry.update();
    }
}
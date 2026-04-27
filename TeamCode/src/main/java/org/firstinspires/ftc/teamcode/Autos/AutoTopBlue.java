
package org.firstinspires.ftc.teamcode.Autos;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous
@Configurable // Panels
public class AutoTopBlue extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private boolean isDone = false;
    private int shootState = 0;
    private Timer pathTimer = new Timer(); // Timer for autonomous path state
    private Timer shootTimer = new Timer(); // Timer for shooting
    private static final double GOAL_X = 7.385694249649367;
    private static final double GOAL_Y = 135.31697054698458;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(14.448300523205479, 112.94468292635028, Math.toRadians(180)));
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        paths = new Paths(follower); // Build paths
        intake.start();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        Pose pose = follower.getPose();
        Vector velocity = follower.getVelocity();
        shooter.update(pose, velocity, GOAL_X, GOAL_Y, telemetry);

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void Shoot() {
        switch (shootState) {
            case 0:

                shootTimer.resetTimer();
                intake.gateOpen();
                isDone = false;
                shootState++;
                break;

            case 1:
                intake.intakeOn();
                shootTimer.resetTimer();
                shootState++;
                break;

            case 2:

                if (shootTimer.getElapsedTimeSeconds() > 0.8) {
                    intake.intakeOff();
                    intake.gateClose();
                    isDone = true;
                }
                break;
        }
    }
    public static class Paths {
        public PathChain Starttopre;
        public PathChain pretomid;
        public PathChain midtoshoot;
        public PathChain shoottogate;
        public PathChain gatetoshoot;
        public PathChain shoottotop;
        public PathChain toptoshoot;
        public PathChain shoottobottom;
        public PathChain bottomtoshoot;

        public Paths(Follower follower) {
            Starttopre = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(14.448, 112.945),
                                    new Pose(47.514, 99.700)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            pretomid = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(47.514, 99.700),
                                    new Pose(69.302, 56.260),
                                    new Pose(10.372, 61.635)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            midtoshoot = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(10.372, 61.635),
                                    new Pose(40.912, 59.214),
                                    new Pose(55.468, 94.721)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            shoottogate = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(55.468, 94.721),
                                    new Pose(45.16830294530154, 58.59957924263672),
                                    new Pose(8.718092566619909, 59.96353436185131)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();

            gatetoshoot = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(9.718092566619909, 59.96353436185131),
                                    new Pose(56.146, 51.905),
                                    new Pose(51.255, 98.881)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            shoottotop = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(51.255, 98.881),
                                    new Pose(55.288, 82.562),
                                    new Pose(13.715, 84.240)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            toptoshoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(13.715, 84.240),
                                    new Pose(58.058, 85.237)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            shoottobottom = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(58.058, 85.237),
                                    new Pose(68.266, 30.375),
                                    new Pose(8.714, 38.620)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            bottomtoshoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(8.714, 38.620),
                                    new Pose(55.436, 109.101)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
        }
    }


    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                    follower.followPath(paths.Starttopre, true);
                    setPathState(1);

                break;

            case 1:

                if (!follower.isBusy()) {
                    Shoot();
                }

                if (isDone) {
                    isDone = false;
                    shootState = 0;
                    intake.intakeOn();
                    follower.followPath(paths.pretomid,true);
                    setPathState(96);

                }
                break;

                case 96:

                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;

            case 2:

                if (pathTimer.getElapsedTimeSeconds() >= 0.2) {
                    intake.intakeOff();
                    follower.followPath(paths.midtoshoot);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    Shoot();
                }

                if (isDone) {
                    follower.followPath(paths.shoottogate, true);
                    setPathState(1000);
                    intake.intakeOn();
                    isDone = false;
                    shootState = 0;
                }
                break;

            case 1000:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;

            case 5:

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(paths.gatetoshoot);
                    intake.intakeOff();
                    setPathState(7);
                }
                break;


            case 7:

                if (!follower.isBusy()) {
                    Shoot();
                }

                if (isDone) {
                    follower.followPath(paths.shoottotop, true);
                    setPathState(99);
                    intake.intakeOn();
                    isDone = false;
                    shootState = 0;
                }
                break;

                case 99:

                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(8);
                }
                break;

            case 8:

                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    intake.intakeOff();
                    follower.followPath(paths.toptoshoot, true);
                    setPathState(9);
                }
                break;

            case 9:

                if (!follower.isBusy()) {
                    Shoot();
                }

                if (isDone) {
                    follower.followPath(paths.shoottobottom, true);
                    setPathState(101);
                    intake.intakeOn();
                    isDone = false;
                    shootState = 0;
                }
                break;

            case 101:

                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(12);
                }
                break;

            case 12:

                if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                    intake.intakeOff();
                    follower.followPath(paths.bottomtoshoot, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    Shoot();
                }

                if (isDone) {
                    setPathState(-1);
                    isDone = false;
                    shootState = 0;
                }
                break;
        }
        return pathState;
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
    
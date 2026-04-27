
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
public class AutoBottomRed extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private boolean isDone = false;
    private int shootState = 0;
    private int loopCounter = 0;
    private Timer pathTimer = new Timer(); // Timer for autonomous path state
    private Timer shootTimer = new Timer(); // Timer for shooting
    private Timer autoTimer = new Timer();
    private static final double GOAL_X = 135.5;
    private static final double GOAL_Y = 134.31697054698458;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(84.85183972924892, 9.91172359956206, Math.toRadians(30)));
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
    @Override
    public void start() {
        autoTimer.resetTimer();
        pathTimer.resetTimer();
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

                if (shootTimer.getElapsedTimeSeconds() > 0.5) {
                    intake.intakeOn();
                    shootTimer.resetTimer();
                    shootState++;
                }

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
        public PathChain Start;
        public PathChain shoot1;
        public PathChain intake2;
        public PathChain shoot2;
        public PathChain intake3;
        public PathChain shoot3;
        public PathChain intake4;
        public PathChain shoot4;
        public PathChain intake5;
        public PathChain shoot5;
        public PathChain park;

        public Paths(Follower follower) {
            Start = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(84.852, 9.912),
                                    new Pose(131.147, 12.451)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                    .build();

            shoot1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(131.147, 12.451),
                                    new Pose(83.261, 12.233)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                    .build();

            intake2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(83.261, 12.233),
                                    new Pose(130.930, 8.855)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(130.930, 8.855),
                                    new Pose(83.261, 12.233)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                    .build();

            intake3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(83.261, 12.233),
                                    new Pose(132.066, 13.451)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(132.066, 13.451),
                                    new Pose(83.261, 12.233)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                    .build();

            intake4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(83.261, 12.233),
                                    new Pose(131.204, 8.948)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                    .build();

            shoot4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(131.204, 8.948),
                                    new Pose(83.261, 12.233)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                    .build();

            intake5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(83.261, 12.233),
                                    new Pose(131.123, 17.516)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                    .build();

            shoot5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(131.123, 17.516),
                                    new Pose(83.261, 12.233)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                    .build();

            park = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(83.261, 12.233),
                                    new Pose(106.461, 9.183)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                    .build();
        }
    }


    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(100);
                }

                break;

            case 100:

                if (!follower.isBusy()) {
                    Shoot();
                }

                if (isDone) {
                    isDone = false;
                    shootState = 0;
                    intake.intakeOn();
                    follower.followPath(paths.Start,true);
                    setPathState(1);

                }
                break;

            case 1:

                if (!follower.isBusy()) {
                    intake.intakeOff();
                    follower.followPath(paths.shoot1, true);
                    setPathState(2);
                }

            case 2:

                if (!follower.isBusy()) {
                    Shoot();
                }

                if (isDone) {
                    follower.followPath(paths.intake2);
                    setPathState(3);
                    intake.intakeOn();
                    isDone = false;
                    shootState = 0;
                }
                break;

            case 3:

                if (!follower.isBusy()) {
                    intake.intakeOff();
                    follower.followPath(paths.shoot2, true);
                    setPathState(4);
                }

                break;

            case 4:
                if (!follower.isBusy()) {
                    Shoot();
                }

                if (isDone) {
                    follower.followPath(paths.intake3);
                    setPathState(5);
                    intake.intakeOn();
                    isDone = false;
                    shootState = 0;
                }
                break;
            case 5:

                if (!follower.isBusy()) {
                    intake.intakeOff();
                    follower.followPath(paths.shoot3, true);
                    setPathState(6);
                }

                break;

            case 6:

                if (!follower.isBusy()) {
                    Shoot();
                }

                if (isDone) {
                    follower.followPath(paths.intake4);
                    setPathState(7);
                    intake.intakeOn();
                    isDone = false;
                    shootState = 0;
                }
                break;

            case 7:

                if (!follower.isBusy()) {
                    intake.intakeOff();
                    follower.followPath(paths.shoot4, true);
                    setPathState(8);
                }

                break;

            case 8:

                if (!follower.isBusy()) {
                    Shoot();
                }

                if (isDone) {
                    follower.followPath(paths.park);
                    setPathState(-1);
                    intake.intakeOff();
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

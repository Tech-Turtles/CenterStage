package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.ARM_LEFT;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.ARM_RIGHT;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.CLAW_LEFT;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.CLAW_RIGHT;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.INTAKE;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.RAMP;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.WRIST;
import static org.firstinspires.ftc.teamcode.opmode.autonomous.SlidePIDTest.kG;
import static org.firstinspires.ftc.teamcode.utility.autonomous.Executive.StateMachine.StateType;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;

import org.firstinspires.ftc.teamcode.hardware.Encoder;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.opmode.autonomous.Autonomous;
import org.firstinspires.ftc.teamcode.opmode.teleop.Manual;
import org.firstinspires.ftc.teamcode.utility.autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.utility.autonomous.Executive;
import org.firstinspires.ftc.teamcode.utility.autonomous.SpikePosition;
import org.firstinspires.ftc.teamcode.utility.autonomous.StartPosition;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.utility.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.utility.pathplanner.controllers.PPHolonomicDriveController;
import org.firstinspires.ftc.teamcode.utility.pathplanner.path.PathPlannerPath;
import org.firstinspires.ftc.teamcode.utility.pathplanner.path.PathPlannerTrajectory;
import org.firstinspires.ftc.teamcode.utility.pathplanner.util.PIDConstants;
import org.opencv.core.Rect;

import java.util.function.Consumer;
import java.util.function.Supplier;

@Config
public class AutonomousStateContext implements Executive.RobotStateMachineContextInterface {

    // Required class variables
    private final Autonomous autonomous;
    private final Executive.StateMachine<Autonomous> stateMachine;
    private final AllianceColor allianceColor;
    private final StartPosition startPosition;
    private PPHolonomicDriveController controller;

    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> speedsSupplier;
    private Consumer<ChassisSpeeds> output;

    private final PathPlannerTrajectory[] trajectories = new PathPlannerTrajectory[6];
    // pixels, x-coordinates
    public static double leftBound = 180, rightBound = 460;
    // seconds
    public static double visionTime = 0.25, spikePlace = 1.0;
    public static SpikePosition spikePosition = SpikePosition.NONE;
    public static boolean centerPark = true;
    public static double parkDelay = 0.0;

    public static RobotConstants.AutonomousRoutine routine = RobotConstants.AutonomousRoutine.SPIKE_PLACE_CYCLE_PARK;
    public AutonomousStateContext(Autonomous autonomous, AllianceColor allianceColor, StartPosition startPosition) {
        this.autonomous = autonomous;
        this.allianceColor = allianceColor;
        this.startPosition = startPosition;
        this.stateMachine = new Executive.StateMachine<>(autonomous);
        stateMachine.update();
    }

    @Override
    public void init() {
        stateMachine.init();

        controller = new PPHolonomicDriveController(
                new PIDConstants(6.0, 0.0, 0.0),
                new PIDConstants(8.0, 0.0, 0.0),
                0.017,
                1.6,
                0.2507
        );

        poseSupplier = autonomous.swerveDrive::getPose;
        speedsSupplier = autonomous.swerveDrive::getRobotVelocity;
        output = autonomous.swerveDrive::drive;

//        loadPathPlannerTrajectories();

        stateMachine.changeState(StateType.DRIVE, new Start());
        stateMachine.changeState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.START));
        stateMachine.changeState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.START, 0.0));
    }

    @Override
    public void update() {
        autonomous.swerveDrive.updateOdometry();
        stateMachine.update();
//        if(packet != null) {
//            Canvas fieldOverlay = packet.fieldOverlay();
//            Executive.StateBase<Autonomous> state = stateMachine.getStateReferenceByType(StateType.DRIVE);
//            if(state.getTrajectory() == null)
//                return;
//
//            fieldOverlay.setStrokeWidth(1);
//            fieldOverlay.setStroke("#4CAF50");
//            DashboardUtil.drawSampledPath(fieldOverlay, state.getTrajectory());
//            double t = state.getTime();
//            Pose2d pose = state.getTrajectory().sample(t).getTargetHolonomicPose();
//            autonomous.telemetry.addData("F Position",pose);
//            DashboardUtil.drawRobot(fieldOverlay, new com.acmerobotics.roadrunner.geometry.Pose2d(pose.getX() / 0.0254, pose.getY() / 0.0254, pose.getRotation().getRadians()));
//
//            fieldOverlay.setStroke("#3F51B5");
//            DashboardUtil.drawPoseHistory(fieldOverlay, autonomous.swerveDrive.getPoseHistory());
//        }
    }

    @Override
    public String getCurrentState() {
        return stateMachine.getCurrentStateByType();
    }

    /**
     * Start State
     * State that sets the robot's position to the start position.
     * Changes the routine based on start position & AutonomousRoutine variable, which can be changed
     * through the dashboard or with a re-install.
     * <p>
     * Trajectory: none
     * Next State: 
     */
    class Start extends Executive.StateBase<Autonomous> {
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            switch (startPosition) {
                case AUDIENCE:
                    //ToDo get start position for red & blue
//                    setStartPosition(new Pose2d());
                    break;
                case BACK_BOARD:
                    if(allianceColor.equals(AllianceColor.BLUE))
                        opMode.swerveDrive.resetOdometry(new Pose2d(new Translation2d(0.215, 2.16535), Rotation2d.fromDegrees(180.0)));
                    else
                        opMode.swerveDrive.resetOdometry(new Pose2d(new Translation2d(3.44, 2.16535), Rotation2d.fromDegrees(0.0)));

                    switch (routine) {
                        case PARK:
                            nextState(StateType.DRIVE, new TopPark(parkDelay));
                            break;
                        case SPIKE_PARK:
                            break;
                        case SPIKE_PLACE_PARK:
                            break;
                        case SPIKE_PLACE_CYCLE_PARK:
                            nextState(StateType.DRIVE, new Detect_Spike());
                            break;
                        default:

                    }
                    break;
                default:
                   throw new IllegalArgumentException("Invalid start position");
            }
        }
        private void setStartPosition(Pose2d pose) {
            opMode.swerveDrive.resetOdometry(pose);
        }
    }

    private class StraightTest extends Executive.StateBase<Autonomous> {
        private boolean hasRun = false, hasDriven = false;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path = PathPlannerPath.fromPathFile("Straight Test");

            trajectory = new PathPlannerTrajectory(path, currentSpeeds);

            controller.reset(poseSupplier.get(), currentSpeeds);
        }

        @Override
        public void update() {
            super.update();
            if(!hasDriven) {
                if (!hasRun) {
                    stateTimer.reset();
                    hasRun = true;
                }
                double currentTime = stateTimer.seconds();
                PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
                Pose2d currentPose = poseSupplier.get();
                ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
                output.accept(targetSpeeds);

                if (currentTime > trajectory.getTotalTimeSeconds()) {
                    hasDriven = true;
                    stateTimer.reset();
                    output.accept(new ChassisSpeeds(0,0,0));
                }
            } else {
                if(opMode.primary.A()) {
                    nextState(StateType.DRIVE, new Back_Test());
                }
            }
        }
    }

    private class TopPark extends Executive.StateBase<Autonomous> {
        private final double delay;
        public TopPark(double delay) {
            this.delay = delay;
        }
        @Override
        public void update() {
            super.update();
            if(stateTimer.seconds() < delay)
                return;
            if(centerPark)
                nextState(StateType.DRIVE, new TopParkCenter());
            else
                nextState(StateType.DRIVE, new TopParkWall());
        }
    }

    private class Back_Test extends Executive.StateBase<Autonomous> {
        private boolean hasRun = false, hasDriven = false;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path = PathPlannerPath.fromPathFile("Back Test");

            trajectory = new PathPlannerTrajectory(path, currentSpeeds);

            controller.reset(poseSupplier.get(), currentSpeeds);
        }

        @Override
        public void update() {
            super.update();
            if(!hasDriven) {
                if (!hasRun) {
                    stateTimer.reset();
                    hasRun = true;
                }
                double currentTime = stateTimer.seconds();
                PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
                Pose2d currentPose = poseSupplier.get();
                ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
                output.accept(targetSpeeds);

                if (currentTime > trajectory.getTotalTimeSeconds()) {
                    hasDriven = true;
                    stateTimer.reset();
                    output.accept(new ChassisSpeeds(0,0,0));
                }
            } else {
                if(opMode.primary.A()) {
                    nextState(StateType.DRIVE, new StraightTest());
                }
            }
        }
    }

    private class TopParkCenter extends Executive.StateBase<Autonomous> {
        private boolean hasRun = false, hasDriven = false;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path = PathPlannerPath.fromPathFile("TopParkCenter");
            trajectory = new PathPlannerTrajectory(path, currentSpeeds);
            controller.reset(poseSupplier.get(), currentSpeeds);
        }

        @Override
        public void update() {
            super.update();
            if(!hasDriven) {
                if (!hasRun) {
                    stateTimer.reset();
                    hasRun = true;
                }
                double currentTime = stateTimer.seconds();
                PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
                Pose2d currentPose = poseSupplier.get();
                ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
                output.accept(targetSpeeds);

                if (currentTime > trajectory.getTotalTimeSeconds()) {
                    hasDriven = true;
                    stateTimer.reset();
                    output.accept(new ChassisSpeeds(0,0,0));
                }
            } else {
                nextState(StateType.DRIVE, new Stop());
            }
        }
    }

    private class TopParkWall extends Executive.StateBase<Autonomous> {
        private boolean hasRun = false, hasDriven = false;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path = PathPlannerPath.fromPathFile("TopParkWall");
            trajectory = new PathPlannerTrajectory(path, currentSpeeds);
            controller.reset(poseSupplier.get(), currentSpeeds);
        }

        @Override
        public void update() {
            super.update();
            if(!hasDriven) {
                if (!hasRun) {
                    stateTimer.reset();
                    hasRun = true;
                }
                double currentTime = stateTimer.seconds();
                PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
                Pose2d currentPose = poseSupplier.get();
                ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
                output.accept(targetSpeeds);

                if (currentTime > trajectory.getTotalTimeSeconds()) {
                    hasDriven = true;
                    stateTimer.reset();
                    output.accept(new ChassisSpeeds(0,0,0));
                }
            } else {
                nextState(StateType.DRIVE, new Stop());
            }
        }
    }

    private class Detect_Spike extends Executive.StateBase<Autonomous> {

        Webcam webcam = RobotConfiguration.WEBCAM.getAsWebcam();

        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);
            nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.HOLD));
            nextState(StateType.SLIDES, new Slide_Position(80));
            nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.GRAB, RobotConstants.ClawOrder.BOTH));
            nextState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.VERTICAL));
        }

        @Override
        public void update() {
            super.update();
//            Rect rect = allianceColor.equals(AllianceColor.BLUE)
//                    ? webcam.getProcessor().getBlueRect()
//                    : webcam.getProcessor().getRedRect();
//
//            try {
//                double x = rect.x + rect.width / 2.0;
//                if (x < leftBound)
//                    spikePosition = SpikePosition.LEFT;
//                else if (x > rightBound)
//                    spikePosition = SpikePosition.RIGHT;
//                else
//                    spikePosition = SpikePosition.MIDDLE;
//            } catch (NullPointerException ignore) {
//            }

            spikePosition = SpikePosition.MIDDLE;

            opMode.telemetry.addData("Spike", spikePosition.name());

            if (stateTimer.seconds() > visionTime) {
                webcam.stop();
                nextState(StateType.DRIVE, new DriveStartPlace());
            }
        }
    }

    private class DriveStartPlace extends Executive.StateBase<Autonomous> {
        private boolean hasRun = false, hasDriven = false, hasSetArm = false, hasSetClaw = false;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;
            if(allianceColor.equals(AllianceColor.BLUE)) {
                if(spikePosition.equals(SpikePosition.LEFT))
                    path = PathPlannerPath.fromPathFile("TopStartToPlaceLeft");
                else if (spikePosition.equals(SpikePosition.RIGHT))
                    path = PathPlannerPath.fromPathFile("TopStartToPlaceRight");
                else
                    path = PathPlannerPath.fromPathFile("TopStartToPlaceMiddle");
            } else {
                if(spikePosition.equals(SpikePosition.LEFT))
                    path = PathPlannerPath.fromPathFile("RedTopStartToPlaceLeft");
                else if (spikePosition.equals(SpikePosition.RIGHT))
                    path = PathPlannerPath.fromPathFile("RedTopStartToPlaceRight");
                else
                    path = PathPlannerPath.fromPathFile("RedTopStartToPlaceMiddle");
            }

            trajectory = new PathPlannerTrajectory(path, currentSpeeds);

            controller.reset(poseSupplier.get(), currentSpeeds);
        }

        @Override
        public void update() {
            super.update();
            if(!hasDriven) {
                if (!hasRun) {
                    stateTimer.reset();
                    hasRun = true;
                }
                double currentTime = stateTimer.seconds();
                PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
                Pose2d currentPose = poseSupplier.get();
                ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
                output.accept(targetSpeeds);

                if(!hasSetArm && stateTimer.seconds() > 1.2) {
                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.BACK_BOARD));
                    hasSetArm = true;
                }

                if (currentTime > trajectory.getTotalTimeSeconds()) {
                    hasDriven = true;
                    stateTimer.reset();
                    output.accept(new ChassisSpeeds(0,0,0));
                }
            } else {
                if(!hasSetClaw && stateTimer.seconds() > 0.2) {
                    nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.OPEN, RobotConstants.ClawOrder.LEFT));
                    hasSetClaw = true;
                }

                if(stateTimer.seconds() > 0.4) {
                    nextState(StateType.DRIVE, new DrivePlaceSpike());
                }
            }
        }
    }

    private class DrivePlaceSpike extends Executive.StateBase<Autonomous> {
        private boolean hasRun = false, hasDriven = false, hasSetArm = false, hasSetClaw = false;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;
            if(allianceColor.equals(AllianceColor.BLUE)) {
                if(spikePosition.equals(SpikePosition.LEFT))
                    path = PathPlannerPath.fromPathFile("TopPlaceLeftToSpikeLeft");
                else if (spikePosition.equals(SpikePosition.RIGHT))
                    path = PathPlannerPath.fromPathFile("TopPlaceRightToSpikeRight");
                else
                    path = PathPlannerPath.fromPathFile("TopPlaceMiddleToSpikeMiddle");
            } else {
                if(spikePosition.equals(SpikePosition.LEFT))
                    path = PathPlannerPath.fromPathFile("RedTopPlaceLeftToSpikeLeft");
                else if (spikePosition.equals(SpikePosition.RIGHT))
                    path = PathPlannerPath.fromPathFile("RedTopPlaceRightToSpikeRight");
                else
                    path = PathPlannerPath.fromPathFile("RedTopPlaceMiddleToSpikeMiddle");
            }

            trajectory = new PathPlannerTrajectory(path, currentSpeeds);

            controller.reset(poseSupplier.get(), currentSpeeds);
        }

        @Override
        public void update() {
            super.update();
            if(!hasDriven) {
                if (!hasRun) {
                    stateTimer.reset();
                    hasRun = true;
                }
                double currentTime = stateTimer.seconds();
                PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
                Pose2d currentPose = poseSupplier.get();
                ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
                output.accept(targetSpeeds);

                if(!hasSetArm && stateTimer.seconds() > 1.5) {
                    nextState(StateType.SLIDES, new Slide_Position(50));
                    nextState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.RIGHT_HORIZONTAL));
                    hasSetArm = true;
                    nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.DRIVE, 0.0));
                }

                if (currentTime > trajectory.getTotalTimeSeconds()) {
                    hasDriven = true;
                    stateTimer.reset();
                    output.accept(new ChassisSpeeds(0,0,0));
                }
            } else {
                if(hasSetArm) {
                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.SPIKE));
                    hasSetArm = false;
                }
                if(!hasSetClaw && stateTimer.seconds() > 0.3) {
                    nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.OPEN, RobotConstants.ClawOrder.RIGHT));
                    hasSetClaw = true;
                }

                if(hasSetClaw && stateTimer.seconds() > 0.5) {
                    nextState(StateType.SLIDES, new Slide_Position(30));
                    nextState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.VERTICAL, 0.15));
                    nextState(StateType.DRIVE, new DriveSpikeCycleGrab());
                }
            }
        }
    }

    private class BottomDriveStartSpike extends Executive.StateBase<Autonomous> {
        private boolean hasRun = false, hasDriven = false, hasSetArm = false, hasSetClaw = false;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;
            if(allianceColor.equals(AllianceColor.BLUE)) {
                if(spikePosition.equals(SpikePosition.LEFT))
                    path = PathPlannerPath.fromPathFile("BottomStartToSpikeLeft");
                else if (spikePosition.equals(SpikePosition.RIGHT))
                    path = PathPlannerPath.fromPathFile("BottomStartToSpikeRight");
                else
                    path = PathPlannerPath.fromPathFile("BottomStartToSpikeMiddle");
            } else {
                if(spikePosition.equals(SpikePosition.LEFT))
                    path = PathPlannerPath.fromPathFile("RedBottomStartToSpikeLeft");
                else if (spikePosition.equals(SpikePosition.RIGHT))
                    path = PathPlannerPath.fromPathFile("RedBottomStartToSpikeRight");
                else
                    path = PathPlannerPath.fromPathFile("RedBottomStartToSpikeMiddle");
            }

            trajectory = new PathPlannerTrajectory(path, currentSpeeds);

            controller.reset(poseSupplier.get(), currentSpeeds);
        }

        @Override
        public void update() {
            super.update();
            if(!hasDriven) {
                if (!hasRun) {
                    stateTimer.reset();
                    hasRun = true;
                }
                double currentTime = stateTimer.seconds();
                PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
                Pose2d currentPose = poseSupplier.get();
                ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
                output.accept(targetSpeeds);

                if(!hasSetArm && stateTimer.seconds() > 1.5) {
                    nextState(StateType.SLIDES, new Slide_Position(50));
                    nextState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.RIGHT_HORIZONTAL));
                    hasSetArm = true;
                    nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.DRIVE, 0.0));
                }

                if (currentTime > trajectory.getTotalTimeSeconds()) {
                    hasDriven = true;
                    stateTimer.reset();
                    output.accept(new ChassisSpeeds(0,0,0));
                }
            } else {
                if(hasSetArm) {
                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.SPIKE));
                    hasSetArm = false;
                }
                if(!hasSetClaw && stateTimer.seconds() > 0.5) {
                    nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.OPEN, RobotConstants.ClawOrder.RIGHT));
                    hasSetClaw = true;
                }

                if(hasSetClaw && stateTimer.seconds() > 1.0) {
                    nextState(StateType.SLIDES, new Slide_Position(30));
                    nextState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.VERTICAL));
                    nextState(StateType.DRIVE, new DriveSpikeCycleGrab());
                }
            }
        }
    }

    private class BottomDrivePlaceSpike extends Executive.StateBase<Autonomous> {
        private boolean hasRun = false, hasDriven = false, hasSetArm = false, hasSetClaw = false;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;
            if(allianceColor.equals(AllianceColor.BLUE)) {
                if(spikePosition.equals(SpikePosition.LEFT))
                    path = PathPlannerPath.fromPathFile("TopSpikeLeftToPlaceLeft");
                else if (spikePosition.equals(SpikePosition.RIGHT))
                    path = PathPlannerPath.fromPathFile("TopPlaceRightToSpikeRight");
                else
                    path = PathPlannerPath.fromPathFile("TopPlaceMiddleToSpikeMiddle");
            } else {
                if(spikePosition.equals(SpikePosition.LEFT))
                    path = PathPlannerPath.fromPathFile("RedTopPlaceLeftToSpikeLeft");
                else if (spikePosition.equals(SpikePosition.RIGHT))
                    path = PathPlannerPath.fromPathFile("RedTopPlaceRightToSpikeRight");
                else
                    path = PathPlannerPath.fromPathFile("RedTopPlaceMiddleToSpikeMiddle");
            }

            trajectory = new PathPlannerTrajectory(path, currentSpeeds);

            controller.reset(poseSupplier.get(), currentSpeeds);
        }

        @Override
        public void update() {
            super.update();
            if(!hasDriven) {
                if (!hasRun) {
                    stateTimer.reset();
                    hasRun = true;
                }
                double currentTime = stateTimer.seconds();
                PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
                Pose2d currentPose = poseSupplier.get();
                ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
                output.accept(targetSpeeds);

                if(!hasSetArm && stateTimer.seconds() > 1.0) {
                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.BACK_BOARD));
                    hasSetArm = true;
                }

                if (currentTime > trajectory.getTotalTimeSeconds()) {
                    hasDriven = true;
                    stateTimer.reset();
                    output.accept(new ChassisSpeeds(0,0,0));
                }
            } else {
                if(!hasSetClaw && stateTimer.seconds() > 0.5) {
                    nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.OPEN, RobotConstants.ClawOrder.LEFT));
                    hasSetClaw = true;
                }

                if(stateTimer.seconds() > 0.75) {
                    nextState(StateType.DRIVE, new DrivePlaceSpike());
                }
            }
        }
    }

    private class DriveSpikeCycleGrab extends Executive.StateBase<Autonomous> {
        private boolean hasRun = false, hasDriven = false, intakeStack = false, hasSetArm = false, bool = false;
        private double delay = 1.8;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;
            if(allianceColor.equals(AllianceColor.BLUE)) {
                if(spikePosition.equals(SpikePosition.LEFT))
                    path = PathPlannerPath.fromPathFile("TopSpikeLeftToCycleGrab");
                else if (spikePosition.equals(SpikePosition.RIGHT))
                    path = PathPlannerPath.fromPathFile("TopSpikeRightToCycleGrab");
                else
                    path = PathPlannerPath.fromPathFile("Brody Path");
            } else {
                if(spikePosition.equals(SpikePosition.LEFT))
                    path = PathPlannerPath.fromPathFile("RedTopSpikeLeftToCycleGrab");
                else if (spikePosition.equals(SpikePosition.RIGHT))
                    path = PathPlannerPath.fromPathFile("RedTopSpikeRightToCycleGrab");
                else
                    path = PathPlannerPath.fromPathFile("RedTopSpikeMiddleToCycleGrab");
            }
            if(spikePosition.equals(SpikePosition.LEFT))
                delay += 1.0;
            else if(spikePosition.equals(SpikePosition.RIGHT))
                delay += 1.5;
            trajectory = new PathPlannerTrajectory(path, currentSpeeds);

            controller.reset(poseSupplier.get(), currentSpeeds);
        }

        @Override
        public void update() {
            super.update();
            if(!hasDriven) {
                if (!hasRun) {
                    stateTimer.reset();
                    hasRun = true;
                }
                double currentTime = stateTimer.seconds();
                PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
                Pose2d currentPose = poseSupplier.get();
                ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
                output.accept(targetSpeeds);

                if(stateTimer.seconds() > delay && !hasSetArm) {
                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.TELEOP_POS));
                    nextState(StateType.SLIDES, new Slide_Position(100));
                    hasSetArm = true;
                }

                if(stateTimer.seconds() > delay + 0.5 && !bool) {
//                    nextState(StateType.SLIDES, new Slide_Speed(-0.5));
                    bool = true;
                }

                if(stateTimer.seconds() > trajectory.getTotalTimeSeconds() - 1.5 && !intakeStack) {
                    nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.STACK, 1.0));
                    intakeStack = true;
                }

                if (currentTime > trajectory.getTotalTimeSeconds()) {
                    hasDriven = true;
                    stateTimer.reset();
                    output.accept(new ChassisSpeeds(0,0,0));
                }
            } else {
                if (stateTimer.seconds() > 0.5) {
                    nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.STACK, 1.0, 0.0));
                    nextState(StateType.DRIVE, new DriveSpikeCyclePlace());
                }
            }
        }
    }

    private class DriveSpikeCyclePlace extends Executive.StateBase<Autonomous> {
        private boolean hasRun = false, hasDriven = false, hasStopIntake = false, bool = false, bool2 = false, bool3 = false, bool4 = false, bool5 = false, bool6 = false, bool7 = false;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;
            if(allianceColor.equals(AllianceColor.BLUE)) {
                path = PathPlannerPath.fromPathFile("TopCycleGrabToCyclePlace");
            } else {
                path = PathPlannerPath.fromPathFile("RedTopCycleGrabToCyclePlace");
            }

            trajectory = new PathPlannerTrajectory(path, currentSpeeds);

            controller.reset(poseSupplier.get(), currentSpeeds);

            nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.OPEN, RobotConstants.ClawOrder.BOTH));
        }

        @Override
        public void update() {
            super.update();
            if(!hasDriven) {
                if (!hasRun) {
                    stateTimer.reset();
                    hasRun = true;
                }
                double currentTime = stateTimer.seconds();
                PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
                Pose2d currentPose = poseSupplier.get();
                ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
                output.accept(targetSpeeds);

                if(stateTimer.seconds() > 0.7 && !hasStopIntake) {
                    nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.INTAKE, -1.0, 0.0));
                    hasStopIntake = true;
                } else if(stateTimer.seconds() > 1.5 && !bool7) {
                    nextState(StateType.SLIDES, new Slide_Position(50));
                    bool7 = true;
                }

                if(stateTimer.seconds() > 1.5 +0.75 && !bool) {
                    nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.DRIVE, 0.0, 0.0));
                    stateMachine.changeState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.DOWN));
                    bool = true;
                } else if(stateTimer.seconds() > 2.0 +0.75 && !bool2 && bool) {
                    stateMachine.changeState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.GRAB, RobotConstants.ClawOrder.BOTH));
                    bool2 = true;
                } else if(stateTimer.seconds() > 2.2 +0.75 && !bool3 && bool2) {
//                    stateMachine.changeState(Executive.StateMachine.StateType.SLIDES, new Slide_Speed(-0.8));
                    bool3 = true;
                } else if(stateTimer.seconds() > 2.3 +0.75 && !bool4 && bool3) {
                    stateMachine.changeState(Executive.StateMachine.StateType.SLIDES, new Slide_Position(200));
                    stateMachine.changeState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.TELEOP_POS));
                    stateMachine.changeState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.RIGHT_HORIZONTAL, 0.8));
                    bool4 = true;
                } else if(stateTimer.seconds() > 2.5 +0.75 && !bool6 && bool4) {
                    stateMachine.changeState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.BACK_BOARD));
                    bool6 = true;
                }

                if (currentTime > trajectory.getTotalTimeSeconds() + .5) {
                    hasDriven = true;
                    stateTimer.reset();
                    output.accept(new ChassisSpeeds(0,0,0));
                }
            } else {
                if(!bool5) {
                    stateMachine.changeState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.OPEN, RobotConstants.ClawOrder.BOTH, 0.25));
                    nextState(StateType.DRIVE, new FinishCycle());
                    bool5 = true;
                }
            }
        }
    }

    private class FinishCycle extends Executive.StateBase<Autonomous> {
        @Override
        public void update() {
            super.update();
            if(stateTimer.seconds() > 0.75 && stateTimer.seconds() < 1.2) {
//                stateMachine.changeState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.TELEOP_POS));
            } else if(stateTimer.seconds() > 1.0)
                stateMachine.changeState(StateType.DRIVE, new CycleGrab2());
        }
    }

    private class CycleGrab2 extends Executive.StateBase<Autonomous> {
        private boolean hasRun = false, hasDriven = false, intakeStack = false, hasSetArm = false, bool = false;
        private double delay = 1.8;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;
            if(allianceColor.equals(AllianceColor.BLUE)) {
                if(spikePosition.equals(SpikePosition.LEFT))
                    path = PathPlannerPath.fromPathFile("TopSpikeLeftToCycleGrab");
                else if (spikePosition.equals(SpikePosition.RIGHT))
                    path = PathPlannerPath.fromPathFile("TopSpikeRightToCycleGrab");
                else
                    path = PathPlannerPath.fromPathFile("CycleGrab2");
            } else {
                if(spikePosition.equals(SpikePosition.LEFT))
                    path = PathPlannerPath.fromPathFile("RedTopSpikeLeftToCycleGrab");
                else if (spikePosition.equals(SpikePosition.RIGHT))
                    path = PathPlannerPath.fromPathFile("RedTopSpikeRightToCycleGrab");
                else
                    path = PathPlannerPath.fromPathFile("RedTopSpikeMiddleToCycleGrab");
            }
            if(spikePosition.equals(SpikePosition.LEFT))
                delay += 1.0;
            else if(spikePosition.equals(SpikePosition.RIGHT))
                delay += 1.5;
            trajectory = new PathPlannerTrajectory(path, currentSpeeds);

            controller.reset(poseSupplier.get(), currentSpeeds);
        }

        @Override
        public void update() {
            super.update();
            if(!hasDriven) {
                if (!hasRun) {
                    stateTimer.reset();
                    hasRun = true;
                }
                double currentTime = stateTimer.seconds();
                PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
                Pose2d currentPose = poseSupplier.get();
                ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
                output.accept(targetSpeeds);

                if(stateTimer.seconds() > delay && !hasSetArm) {
                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.TELEOP_POS));
                    stateMachine.changeState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.VERTICAL));
                    nextState(StateType.SLIDES, new Slide_Speed(-0.5));
                    hasSetArm = true;
                }

                if(stateTimer.seconds() > delay + 0.5 && !bool) {
                    nextState(StateType.SLIDES, new Slide_Speed(-0.5));
                    bool = true;
                }

                if(stateTimer.seconds() > trajectory.getTotalTimeSeconds() - 1.5 && !intakeStack) {
                    nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.INTAKE, 1.0));
                    intakeStack = true;
                }

                if (currentTime > trajectory.getTotalTimeSeconds()) {
                    hasDriven = true;
                    stateTimer.reset();
                    output.accept(new ChassisSpeeds(0,0,0));
                    nextState(StateType.SLIDES, new Slide_Position(100));
                }
            } else {
                if (stateTimer.seconds() > 0.5) {
                    nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.INTAKE, 1.0, 0.0));
                    nextState(StateType.DRIVE, new CyclePlace2());
                }
            }
        }
    }

    private class CyclePlace2 extends Executive.StateBase<Autonomous> {
        private boolean hasRun = false, hasDriven = false, hasStopIntake = false, bool = false, bool2 = false, bool3 = false, bool4 = false, bool5 = false, bool6 = false, bool7 = false;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;
            if(allianceColor.equals(AllianceColor.BLUE)) {
                path = PathPlannerPath.fromPathFile("TopCycleGrabToCyclePlace");
            } else {
                path = PathPlannerPath.fromPathFile("RedTopCycleGrabToCyclePlace");
            }

            trajectory = new PathPlannerTrajectory(path, currentSpeeds);

            controller.reset(poseSupplier.get(), currentSpeeds);

            nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.OPEN, RobotConstants.ClawOrder.BOTH));
        }

        @Override
        public void update() {
            super.update();
            if(!hasDriven) {
                if (!hasRun) {
                    stateTimer.reset();
                    hasRun = true;
                }
                double currentTime = stateTimer.seconds();
                PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
                Pose2d currentPose = poseSupplier.get();
                ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
                output.accept(targetSpeeds);

                if(stateTimer.seconds() > 0.7 && !hasStopIntake) {
                    nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.INTAKE, -1.0, 0.0));
                    hasStopIntake = true;
                } else if(stateTimer.seconds() > 1.5 && !bool7) {
                    nextState(StateType.SLIDES, new Slide_Position(50));
                    bool7 = true;
                }

                if(stateTimer.seconds() > 1.5 +0.75 && !bool) {
                    nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.DRIVE, 0.0, 0.0));
                    stateMachine.changeState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.DOWN));
                    bool = true;
                } else if(stateTimer.seconds() > 2.0 +0.75 && !bool2 && bool) {
                    stateMachine.changeState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.GRAB, RobotConstants.ClawOrder.BOTH));
                    bool2 = true;
                } else if(stateTimer.seconds() > 2.2 +0.75 && !bool3 && bool2) {
//                    stateMachine.changeState(Executive.StateMachine.StateType.SLIDES, new Slide_Speed(-0.8));
                    bool3 = true;
                } else if(stateTimer.seconds() > 2.3 +0.75 && !bool4 && bool3) {
                    stateMachine.changeState(Executive.StateMachine.StateType.SLIDES, new Slide_Position(200));
                    stateMachine.changeState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.TELEOP_POS));
                    stateMachine.changeState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.RIGHT_HORIZONTAL, 0.8));
                    bool4 = true;
                } else if(stateTimer.seconds() > 2.5 +0.75 && !bool6 && bool4) {
                    stateMachine.changeState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.BACK_BOARD));
                    bool6 = true;
                }

                if (currentTime > trajectory.getTotalTimeSeconds() + .2) {
                    hasDriven = true;
                    stateTimer.reset();
                    output.accept(new ChassisSpeeds(0,0,0));
                }
            } else {
                if(!bool5) {
                    stateMachine.changeState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.OPEN, RobotConstants.ClawOrder.BOTH, 0.2));
                    nextState(StateType.DRIVE, new FinishCycle2());
                    bool5 = true;
                }
            }
        }
    }

    private class FinishCycle2 extends Executive.StateBase<Autonomous> {
        private boolean hasRun = false, hasDriven = false;

        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path = PathPlannerPath.fromPathFile("Brody Path 2");

            trajectory = new PathPlannerTrajectory(path, currentSpeeds);

            controller.reset(poseSupplier.get(), currentSpeeds);
        }

        @Override
        public void update() {
            super.update();
            if(!hasDriven) {
                if (!hasRun) {
                    stateTimer.reset();
                    hasRun = true;
                }
                double currentTime = stateTimer.seconds();
                PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
                Pose2d currentPose = poseSupplier.get();
                ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
                output.accept(targetSpeeds);

                if (currentTime > trajectory.getTotalTimeSeconds()) {
                    hasDriven = true;
                    stateTimer.reset();
                    output.accept(new ChassisSpeeds(0,0,0));
//                    stateMachine.changeState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.TELEOP_POS));
//                    stateMachine.changeState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.VERTICAL));
                }
            } else {
                if(stateTimer.seconds() > 0.25) {
                    stateMachine.changeState(StateType.DRIVE, new Stop());
                }
            }
        }
    }

    class Slide_Speed extends Executive.StateBase<Autonomous> {
        private final Motor left = RobotConfiguration.SLIDE_LEFT.getAsMotor(), right = RobotConfiguration.SLIDE_RIGHT.getAsMotor();
        private final double speed;
        Slide_Speed(double speed) {
            this.speed = speed;
        }

        @Override
        public void update() {
            super.update();
            left.setPower(speed);
            right.setPower(speed);
        }
    }

    private class Audience_Park extends Executive.StateBase<Autonomous> {
        
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;
            if(allianceColor.equals(AllianceColor.BLUE)) {
                path = PathPlannerPath.fromPathFile("BlueAudiencePark");
                opMode.swerveDrive.resetOdometry(new Pose2d(new Translation2d(0.23, 0.91), Rotation2d.fromDegrees(180.0)));
            } else {
                path = PathPlannerPath.fromPathFile("RedBottomPark");
                opMode.swerveDrive.resetOdometry(new Pose2d(new Translation2d(3.36, 0.91), Rotation2d.fromDegrees(0.0)));
            }
            trajectory = new PathPlannerTrajectory(path, currentSpeeds);

            controller.reset(opMode.swerveDrive.getPose(), new ChassisSpeeds(0.0, 0.0, 0.0));
            Pose2d currentPose = poseSupplier.get();

            controller.reset(currentPose, currentSpeeds);
        }

        @Override
        public void update() {
            super.update();
            double currentTime = stateTimer.seconds();
            PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
            Pose2d currentPose = poseSupplier.get();
            output.accept(controller.calculateRobotRelativeSpeeds(currentPose, targetState));

            if (currentTime > trajectory.getTotalTimeSeconds())
                nextState(StateType.DRIVE, new Stop());
        }
    }

    private class Back_Park extends Executive.StateBase<Autonomous> {
        
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;
            if(allianceColor.equals(AllianceColor.BLUE)) {
                path = PathPlannerPath.fromPathFile("BlueBackPark");
                opMode.swerveDrive.resetOdometry(new Pose2d(new Translation2d(0.23, 2.13), Rotation2d.fromDegrees(180.0)));
            } else {
                path = PathPlannerPath.fromPathFile("RedBackPark");
                opMode.swerveDrive.resetOdometry(new Pose2d(new Translation2d(3.20, 2.14), Rotation2d.fromDegrees(0.0)));
            }
            trajectory = new PathPlannerTrajectory(path, currentSpeeds);

            controller.reset(opMode.swerveDrive.getPose(), new ChassisSpeeds(0.0, 0.0, 0.0));
            Pose2d currentPose = poseSupplier.get();

            controller.reset(currentPose, currentSpeeds);
        }

        @Override
        public void update() {
            super.update();
            double currentTime = stateTimer.seconds();
            PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
            Pose2d currentPose = poseSupplier.get();
            output.accept(controller.calculateRobotRelativeSpeeds(currentPose, targetState));

            if (currentTime > trajectory.getTotalTimeSeconds())
                nextState(StateType.DRIVE, new Stop());
        }
    }

    private static class Slide_Position extends Executive.StateBase<Autonomous> {

        private final Motor left = RobotConfiguration.SLIDE_LEFT.getAsMotor(), right = RobotConfiguration.SLIDE_RIGHT.getAsMotor();
        private final Encoder slides = RobotConfiguration.LIFT_ENCODER.getAsEncoder();
        private final double setpoint, delay;
        private boolean generate = false;
        private MotionProfile activeProfile;
        private double profileStart;
        private final PIDFController controller = RobotConstants.slideController;


        Slide_Position(double setpoint) {
            this(setpoint, 0.0);
        }

        Slide_Position(double setpoint, double delay) {
            this.setpoint = setpoint;
            this.delay = delay;
        }

        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);
            profileStart = stateTimer.seconds();
            MotionState start = new MotionState(slides.getCurrentPosition(), 0, 0, 0);
            MotionState goal = new MotionState(setpoint, 0, 0, 0);
            activeProfile = MotionProfileGenerator.generateSimpleMotionProfile(start, goal, 1400, 900);
            generate = delay > 0.0;
        }

        @Override
        public void update() {
            super.update();
            if(delay > 0.0 && !generate) {
                if(stateTimer.seconds() > delay) {
                    profileStart = stateTimer.seconds();
                    MotionState start = new MotionState(slides.getCurrentPosition(), 0, 0, 0);
                    MotionState goal = new MotionState(setpoint, 0, 0, 0);
                    activeProfile = MotionProfileGenerator.generateSimpleMotionProfile(start, goal, 1400, 900);
                    generate = true;
                }
                return;
            }
            double profileTime = stateTimer.seconds() - profileStart;

            MotionState motionState = activeProfile.get(profileTime);

            controller.setTargetPosition(motionState.getX());
            controller.setTargetVelocity(motionState.getV());
            controller.setTargetAcceleration(motionState.getA());
            double power = controller.update(slides.getCurrentPosition(), slides.getRawVelocity());
            power = power > 0.01 ? power + kG : power;
            left.setPower(power);
            right.setPower(power);

            isDone = (stateTimer.seconds() - profileStart) > activeProfile.duration();
        }
    }

    private static class Claw_Position extends Executive.StateBase<Autonomous> {
        private final RobotConstants.ClawPosition clawPosition;
        private final RobotConstants.ClawOrder order;
        private final double delay;
        private final Servo clawLeft = CLAW_LEFT.getAsServo(), clawRight = CLAW_RIGHT.getAsServo();

        Claw_Position(RobotConstants.ClawPosition clawPosition, RobotConstants.ClawOrder order) {
            this(clawPosition, order, 0.0);
        }

        Claw_Position(RobotConstants.ClawPosition clawPosition, RobotConstants.ClawOrder order, double delay) {
            this.clawPosition = clawPosition;
            this.order = order;
            this.delay = delay;
        }

        @Override
        public void update() {
            super.update();

            if(stateTimer.seconds() < delay)
                return;

            switch (order) {
                case LEFT:
                    clawLeft.setPosition(clawPosition.getLeftPos());
                    break;
                case RIGHT:
                    clawRight.setPosition(clawPosition.getRightPos());
                    break;
                case BOTH:
                    clawLeft.setPosition(clawPosition.getLeftPos());
                    clawRight.setPosition(clawPosition.getRightPos());
            }
        }
    }

    private static class Arm_Position extends Executive.StateBase<Autonomous> {
        private final double leftPos, rightPos, delay;
        private final Servo armLeft = ARM_LEFT.getAsServo(), armRight = ARM_RIGHT.getAsServo();

        Arm_Position(RobotConstants.ArmPosition armPosition) {
            this(armPosition.getLeftPos(), armPosition.getRightPos(), 0.0);
        }

        Arm_Position(RobotConstants.ArmPosition armPosition, double delay) {
            this(armPosition.getLeftPos(), armPosition.getRightPos(), delay);
        }

        Arm_Position(double leftPos, double rightPos) {
            this(leftPos, rightPos, 0.0);
        }

        Arm_Position(double leftPos, double rightPos, double delay) {
            this.leftPos = leftPos;
            this.rightPos = rightPos;
            this.delay = delay;
        }

        @Override
        public void update() {
            super.update();

            if(stateTimer.seconds() < delay)
                return;

            armLeft.setPosition(leftPos);
            armRight.setPosition(rightPos);
        }
    }

    private static class Wrist_Position extends Executive.StateBase<Autonomous> {
        private final double pos, delay;
        private final Servo wrist = RobotConfiguration.WRIST.getAsServo();

        Wrist_Position(RobotConstants.WristPosition wristPosition) {
            this(wristPosition.getPosition(), 0.0);
        }

        Wrist_Position(RobotConstants.WristPosition wristPosition, double delay) {
            this(wristPosition.getPosition(), delay);
        }

        Wrist_Position(double pos) {
            this(pos, 0.0);
        }

        Wrist_Position(double pos, double delay) {
            this.pos = pos;
            this.delay = delay;
        }

        @Override
        public void update() {
            super.update();

            if(stateTimer.seconds() < delay)
                return;
            wrist.setPosition(pos);
        }
    }

    /**
     * State to control the intake's ramp angle & the intake's motor power.
     * Optional delay in case the state needs to wait for some amount of seconds.
     */
    private static class Intake_Position extends Executive.StateBase<Autonomous> {
        private final double pos, power, delay;

        Intake_Position(RobotConstants.IntakePosition intakePosition, double power) {
            this(intakePosition.getPosition(), power, 0.0);
        }

        Intake_Position(RobotConstants.IntakePosition intakePosition, double power, double delay) {
            this(intakePosition.getPosition(), power, delay);
        }

        Intake_Position(double pos, double power) {
            this(pos, power, 0.0);
        }

        Intake_Position(double pos, double power, double delay) {
            this.pos = pos;
            this.power = power;
            this.delay = delay;
        }

        @Override
        public void update() {
            super.update();

            RAMP.getAsServo().setPosition(pos);
            if(stateTimer.seconds() < delay)
                return;
            INTAKE.getAsMotor().setPower(power);
        }
    }

    private static class Stop extends Executive.StateBase<Autonomous> {
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);
            for (Executive.StateMachine.StateType type : Executive.StateMachine.StateType.values())
                stateMachine.removeStateByType(type);
            opMode.stop();
        }
    }

    private static class StopMotors extends  Executive.StateBase<Autonomous> {
        private final Motor[] motors;

        StopMotors(Motor... motors) {
            this.motors = motors;
        }

        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);
            for (Motor motor : motors)
                motor.setPower(0.0);
        }
    }

    private void loadPathPlannerTrajectories() {
//        trajectories[RobotConstants.AutonomousRoutine.PARK.ordinal()] = new PathPlannerTrajectory(PathPlannerPath.fromPathFile(RobotConstants.BACK_PARK), new ChassisSpeeds());

    }
}
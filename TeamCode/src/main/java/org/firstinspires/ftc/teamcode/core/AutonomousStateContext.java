package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.ARM_LEFT;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.ARM_RIGHT;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.CLAW_LEFT;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.CLAW_RIGHT;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.INTAKE;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.RAMP;
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
import org.firstinspires.ftc.teamcode.opmode.autonomous.Autonomous;
import org.firstinspires.ftc.teamcode.opmode.autonomous.TransferTest;
import org.firstinspires.ftc.teamcode.utility.autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.utility.autonomous.Executive;
import org.firstinspires.ftc.teamcode.utility.autonomous.SpikePosition;
import org.firstinspires.ftc.teamcode.utility.autonomous.StartPosition;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utility.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.utility.pathplanner.controllers.PPHolonomicDriveController;
import org.firstinspires.ftc.teamcode.utility.pathplanner.path.PathPlannerPath;
import org.firstinspires.ftc.teamcode.utility.pathplanner.path.PathPlannerTrajectory;
import org.firstinspires.ftc.teamcode.utility.pathplanner.util.PIDConstants;

import java.util.function.Consumer;
import java.util.function.Supplier;

@Config
public class AutonomousStateContext implements Executive.RobotStateMachineContextInterface {

    // Required class variables
    private final Autonomous autonomous;
    private final Executive.StateMachine<Autonomous> stateMachine;
    private final AllianceColor allianceColor;
    private StartPosition startPosition;
    private PPHolonomicDriveController controller;

    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> speedsSupplier;
    private Consumer<ChassisSpeeds> output;

    private final PathPlannerTrajectory[] trajectories = new PathPlannerTrajectory[6];
    // pixels, x-coordinates
    public static double leftBound = 180, rightBound = 460;
    // seconds
    public static double visionTime = 0.25, spikePlace = 1.0;
    public static double LOCALIZE_Y = 3.9; // meters
    public static SpikePosition spikePosition = SpikePosition.MIDDLE;
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
        autonomous.swerveDrive.updateIMU();
        autonomous.swerveDrive.updateOdometry();
        stateMachine.update();
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

            startPosition = StartPosition.AUDIENCE;

            switch (startPosition) {
                case AUDIENCE:
                    if(allianceColor.equals(AllianceColor.BLUE))
                        setStartPosition(RobotConstants.BLUE_AUDIENCE_START);
                    else
                        setStartPosition(RobotConstants.RED_AUDIENCE_START);

                    switch (routine) {
                        case PARK:
                        case SPIKE_PARK:
                        case SPIKE_PLACE_PARK:
                        case SPIKE_PLACE_CYCLE_PARK:
                            nextState(StateType.DRIVE, new FarCenterToSpike());
                    }
                    break;
                case CENTER:
                    if(allianceColor.equals(AllianceColor.BLUE))
                        setStartPosition(RobotConstants.BLUE_CENTER_START);
                    else
                        setStartPosition(RobotConstants.RED_CENTER_START);

                    switch (routine) {
                        case PARK:
                        case SPIKE_PARK:
                        case SPIKE_PLACE_PARK:
                        case SPIKE_PLACE_CYCLE_PARK:
                            nextState(StateType.DRIVE, new StartCenterToSpike());
                    }
                    break;
                case BACK_BOARD:
                    if(allianceColor.equals(AllianceColor.BLUE))
                        setStartPosition(RobotConstants.BLUE_BACKBOARD_START);
                    else
                        setStartPosition(RobotConstants.RED_BACKBOARD_START);

                    switch (routine) {
                        case PARK:
                            nextState(StateType.DRIVE, new StartBackboardToPark(parkDelay));
                            break;
                        case SPIKE_PARK:
                        case SPIKE_PLACE_PARK:
                        case SPIKE_PLACE_CYCLE_PARK:
                            nextState(StateType.DRIVE, new StartBackBoardToSpike());
                            break;
                        default:
                    }
                    break;
                default:
                   throw new IllegalArgumentException("Invalid start position");
            }

            nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.HOLD));
            nextState(StateType.SLIDES, new Slide_Position(80));
            nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.GRAB, RobotConstants.ClawOrder.BOTH));
            nextState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.VERTICAL));
        }
        private void setStartPosition(Pose2d pose) {
            opMode.swerveDrive.resetOdometry(pose);
        }
    }

    private class StartBackboardToPark extends Executive.DrivingStateBase<Autonomous> {
        private final double delay;
        public StartBackboardToPark() {
            this(0.0);
        }
        public StartBackboardToPark(double delay) {
            this.delay = delay;
        }

        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

        }
    }

    private class StartBackBoardToSpike extends Executive.DrivingStateBase<Autonomous> {
        boolean hasSetArm = false, hasDriven = false, hasSetClaw = false;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;

            if(spikePosition.equals(SpikePosition.LEFT))
                path = PathPlannerPath.fromPathFile("Blue left spike place 1");
            else if (spikePosition.equals(SpikePosition.RIGHT))
                path = PathPlannerPath.fromPathFile("Blue Left Spike 3 place");
            else
                path = PathPlannerPath.fromPathFile("Blue Left Spike Place 2");

            if(allianceColor.equals(AllianceColor.RED)) {
                path = path.flipPath();
            }

            trajectory = new PathPlannerTrajectory(path, currentSpeeds, poseSupplier.get().getRotation());

            controller.reset(poseSupplier.get(), currentSpeeds);

            nextState(StateType.SLIDES, new Slide_Position(50));
            nextState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.RIGHT_HORIZONTAL));
            nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.DRIVE, 0.0));
        }

        @Override
        public void update() {
            super.update();

            double currentTime = drivingTimer.seconds();
            PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
            Pose2d currentPose = poseSupplier.get();
            ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
            output.accept(targetSpeeds);

            if(!hasSetArm && drivingTimer.seconds() > 0.2) {
                nextState(StateType.ARM, new Arm_Position(0.28, 0.28));
                hasSetArm = true;
            }

            if (currentTime > finishTime && !hasDriven) {
                hasDriven = true;
                stateTimer.reset();
                output.accept(new ChassisSpeeds(0,0,0));
            }

            if(hasDriven) {
                if(!hasSetClaw && stateTimer.seconds() > 0.3) {
                    nextState(StateType.ARM, new Arm_Position(0.25, 0.25));
                    nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.OPEN, RobotConstants.ClawOrder.RIGHT, 0.1));
                    hasSetClaw = true;
                }

                if(hasSetClaw && stateTimer.seconds() > 0.6) {
                    nextState(StateType.SLIDES, new Slide_Position(100));
                    nextState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.RIGHT_HORIZONTAL, 0.15));
                    nextState(StateType.ARM, new Arm_Position(0.3, 0.3));
                    nextState(StateType.DRIVE, new BackBoardSpikeToPlace());
                }
            }
        }
    }

    private class BackBoardSpikeToPlace extends Executive.DrivingStateBase<Autonomous> {
        boolean hasSetArm = false, hasDriven = false, hasSetClaw = false;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;

            if(spikePosition.equals(SpikePosition.LEFT))
                path = PathPlannerPath.fromPathFile("Blue Left Spike 1 Backboard Place");
            else if (spikePosition.equals(SpikePosition.RIGHT))
                path = PathPlannerPath.fromPathFile("Blue left spike 3 backboard");
            else
                path = PathPlannerPath.fromPathFile("Blue Left Spike 2 Backboard");

            if(allianceColor.equals(AllianceColor.RED)) {
                path = path.flipPath();
            }

            trajectory = new PathPlannerTrajectory(path, currentSpeeds, poseSupplier.get().getRotation());

            controller.reset(poseSupplier.get(), currentSpeeds);

            nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.BACK_BOARD));
        }

        @Override
        public void update() {
            super.update();

            if(stateTimer.seconds() > 0.5 || hasDriven) {
                if(!hasSetArm) {
                    hasSetArm = true;
                    drivingTimer.reset();
                }
            } else
                return;

            double currentTime = drivingTimer.seconds();
            PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
            Pose2d currentPose = poseSupplier.get();
            ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
            output.accept(targetSpeeds);

            if (currentTime > finishTime && !hasDriven) {
                hasDriven = true;
                stateTimer.reset();
                output.accept(new ChassisSpeeds(0,0,0));
            }

            if(hasDriven) {
                if(!hasSetClaw && stateTimer.seconds() > 0.3) {
                    nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.OPEN, RobotConstants.ClawOrder.LEFT, 0.1));
                    hasSetClaw = true;
                }

                if(hasSetClaw && stateTimer.seconds() > 0.6) {
                    nextState(StateType.SLIDES, new Slide_Position(80));
                    nextState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.VERTICAL, 0.15));
                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.TELEOP_POS, 0.1));
                    nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.DRIVE, 0.0));
                    nextState(StateType.DRIVE, new BackBoardPlaceToPark());
                }
            }
        }
    }

    private class BackBoardPlaceToPark extends Executive.DrivingStateBase<Autonomous> {
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;

            if(spikePosition.equals(SpikePosition.LEFT))
                path = PathPlannerPath.fromPathFile("blue left spike 1 park");
            else if (spikePosition.equals(SpikePosition.RIGHT))
                path = PathPlannerPath.fromPathFile("Blue left 3 park");
            else
                path = PathPlannerPath.fromPathFile("Blue Left Spike 2 Park");

            if(allianceColor.equals(AllianceColor.RED)) {
                path = path.flipPath();
            }

            trajectory = new PathPlannerTrajectory(path, currentSpeeds, poseSupplier.get().getRotation());

            controller.reset(poseSupplier.get(), currentSpeeds);
        }

        @Override
        public void update() {
            super.update();

            double currentTime = drivingTimer.seconds();
            PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
            Pose2d currentPose = poseSupplier.get();
            ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
            output.accept(targetSpeeds);

            if (currentTime > finishTime) {
                output.accept(new ChassisSpeeds(0,0,0));
                nextState(StateType.DRIVE, new Stop());
            }
        }
    }

    private class StartCenterToSpike extends Executive.DrivingStateBase<Autonomous> {
        boolean hasSetArm = false, hasDriven = false, hasSetClaw = false;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;

            if(spikePosition.equals(SpikePosition.LEFT))
                path = PathPlannerPath.fromPathFile("Blue center 1 spike");
            else if (spikePosition.equals(SpikePosition.RIGHT))
                path = PathPlannerPath.fromPathFile("Blue center 3 spike");
            else
                path = PathPlannerPath.fromPathFile("Blue center 2 spike");

            if(allianceColor.equals(AllianceColor.RED)) {
                path = path.flipPath();
            }

            trajectory = new PathPlannerTrajectory(path, currentSpeeds, poseSupplier.get().getRotation());

            controller.reset(poseSupplier.get(), currentSpeeds);

            nextState(StateType.SLIDES, new Slide_Position(50));
            nextState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.RIGHT_HORIZONTAL));
            nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.DRIVE, 0.0));
        }

        @Override
        public void update() {
            super.update();

            double currentTime = drivingTimer.seconds();
            PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
            Pose2d currentPose = poseSupplier.get();
            ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
            output.accept(targetSpeeds);

            if(!hasSetArm && drivingTimer.seconds() > 0.2) {
                nextState(StateType.ARM, new Arm_Position(0.28, 0.28));
                hasSetArm = true;
            }

            if (currentTime > finishTime && !hasDriven) {
                hasDriven = true;
                stateTimer.reset();
                output.accept(new ChassisSpeeds(0,0,0));
            }

            if(hasDriven) {
                if(!hasSetClaw && stateTimer.seconds() > 0.3) {
                    nextState(StateType.ARM, new Arm_Position(0.25, 0.25));
                    nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.OPEN, RobotConstants.ClawOrder.RIGHT, 0.1));
                    hasSetClaw = true;
                }

                if(hasSetClaw && stateTimer.seconds() > 0.6) {
                    nextState(StateType.SLIDES, new Slide_Position(100));
                    nextState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.RIGHT_HORIZONTAL, 0.15));
                    nextState(StateType.ARM, new Arm_Position(0.3, 0.3));
                    nextState(StateType.DRIVE, new CenterSpikeToPlace());
                }
            }
        }
    }

    private class CenterSpikeToPlace extends Executive.DrivingStateBase<Autonomous> {
        boolean hasSetArm = false, hasDriven = false, hasSetClaw = false;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;

            if(spikePosition.equals(SpikePosition.LEFT))
                path = PathPlannerPath.fromPathFile("blue center 1 backboard place");
            else if (spikePosition.equals(SpikePosition.RIGHT))
                path = PathPlannerPath.fromPathFile("blue center 3 backboard place");
            else
                path = PathPlannerPath.fromPathFile("blue center 2 backboard place");

            if(allianceColor.equals(AllianceColor.RED)) {
                path = path.flipPath();
            }

            trajectory = new PathPlannerTrajectory(path, currentSpeeds, poseSupplier.get().getRotation());

            controller.reset(poseSupplier.get(), currentSpeeds);

            nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.BACK_BOARD));
        }

        @Override
        public void update() {
            super.update();

            if(stateTimer.seconds() > 0.5 || hasDriven) {
                if(!hasSetArm) {
                    hasSetArm = true;
                    drivingTimer.reset();
                }
            } else
                return;

            double currentTime = drivingTimer.seconds();
            PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
            Pose2d currentPose = poseSupplier.get();
            ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
            output.accept(targetSpeeds);

            if (currentTime > finishTime && !hasDriven) {
                hasDriven = true;
                stateTimer.reset();
                output.accept(new ChassisSpeeds(0,0,0));
            }

            if(hasDriven) {
                if(!hasSetClaw && stateTimer.seconds() > 0.3) {
                    nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.OPEN, RobotConstants.ClawOrder.LEFT, 0.1));
                    hasSetClaw = true;
                }

                if(hasSetClaw && stateTimer.seconds() > 0.6) {
                    nextState(StateType.SLIDES, new Slide_Position(80));
                    nextState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.VERTICAL, 0.15));
                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.TELEOP_POS, 0.1));
                    nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.DRIVE, 0.0));
                    nextState(StateType.DRIVE, new CenterPlaceToPark());
                }
            }
        }
    }

    private class CenterPlaceToPark extends Executive.DrivingStateBase<Autonomous> {
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;

            if(spikePosition.equals(SpikePosition.LEFT))
                path = PathPlannerPath.fromPathFile("blue center 1 park");
            else if (spikePosition.equals(SpikePosition.RIGHT))
                path = PathPlannerPath.fromPathFile("blue center 3 park");
            else
                path = PathPlannerPath.fromPathFile("blue center 2 park");

            if(allianceColor.equals(AllianceColor.RED)) {
                path = path.flipPath();
            }

            trajectory = new PathPlannerTrajectory(path, currentSpeeds, poseSupplier.get().getRotation());

            controller.reset(poseSupplier.get(), currentSpeeds);
        }

        @Override
        public void update() {
            super.update();

            double currentTime = drivingTimer.seconds();
            PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
            Pose2d currentPose = poseSupplier.get();
            ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
            output.accept(targetSpeeds);

            if (currentTime > finishTime) {
                output.accept(new ChassisSpeeds(0,0,0));
                nextState(StateType.DRIVE, new Stop());
            }
        }
    }

    private class FarCenterToSpike extends Executive.DrivingStateBase<Autonomous> {
        boolean hasSetArm = false, hasDriven = false, hasSetClaw = false;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;

            if(spikePosition.equals(SpikePosition.LEFT))
                path = PathPlannerPath.fromPathFile("Blue Right Spike place 1");
            else if (spikePosition.equals(SpikePosition.RIGHT))
                path = PathPlannerPath.fromPathFile("Blue Right Spike place 3");
            else
                path = PathPlannerPath.fromPathFile("Blue Right Spike place 2");

            if(allianceColor.equals(AllianceColor.RED)) {
                path = path.flipPath();
            }

            trajectory = new PathPlannerTrajectory(path, currentSpeeds, poseSupplier.get().getRotation());

            controller.reset(poseSupplier.get(), currentSpeeds);

            nextState(StateType.SLIDES, new Slide_Position(50));
            nextState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.RIGHT_HORIZONTAL));
            nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.DRIVE, 0.0));
        }

        @Override
        public void update() {
            super.update();

            double currentTime = drivingTimer.seconds();
            PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
            Pose2d currentPose = poseSupplier.get();
            ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
            output.accept(targetSpeeds);

            if(!hasSetArm && drivingTimer.seconds() > 0.2) {
                nextState(StateType.ARM, new Arm_Position(0.28, 0.28));
                hasSetArm = true;
            }

            if (currentTime > finishTime && !hasDriven) {
                hasDriven = true;
                stateTimer.reset();
                output.accept(new ChassisSpeeds(0,0,0));
            }

            if(hasDriven) {
                if(!hasSetClaw && stateTimer.seconds() > 0.3) {
                    nextState(StateType.ARM, new Arm_Position(0.25, 0.25));
                    nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.OPEN, RobotConstants.ClawOrder.RIGHT, 0.1));
                    hasSetClaw = true;
                }

                if(hasSetClaw && stateTimer.seconds() > 0.6) {
                    nextState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.VERTICAL, 0.05));
                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.TELEOP_POS));
                    nextState(StateType.SLIDES, new Slide_Position(10));
                    nextState(StateType.DRIVE, new FarSpikeToBackBoard());
                }
            }
        }
    }

//    private class FarSpikeToGrab extends Executive.DrivingStateBase<Autonomous> {
//        boolean hasDriven = false, hasSpit = false;
//        @Override
//        public void init(Executive.StateMachine<Autonomous> stateMachine) {
//            super.init(stateMachine);
//
//            ChassisSpeeds currentSpeeds = speedsSupplier.get();
//            PathPlannerPath path;
//
//            if(spikePosition.equals(SpikePosition.LEFT))
//                path = PathPlannerPath.fromPathFile("Blue Right Spike 1 Grab 1 Pixel");
//            else if (spikePosition.equals(SpikePosition.RIGHT))
//                path = PathPlannerPath.fromPathFile("stack grab 3");
//            else
//                path = PathPlannerPath.fromPathFile("New Path");
//
//            if(allianceColor.equals(AllianceColor.RED)) {
//                path = path.flipPath();
//            }
//
//            trajectory = new PathPlannerTrajectory(path, currentSpeeds, poseSupplier.get().getRotation());
//
//            controller.reset(poseSupplier.get(), currentSpeeds);
//
//            nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.BACK_BOARD));
//            nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.STACK, 1.0, 1.0));
//        }
//
//        @Override
//        public void update() {
//            super.update();
//
//            double currentTime = drivingTimer.seconds();
//            PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
//            Pose2d currentPose = poseSupplier.get();
//            ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
//            output.accept(targetSpeeds);
//
//            if (currentTime > finishTime && !hasDriven) {
//                hasDriven = true;
//                stateTimer.reset();
//                output.accept(new ChassisSpeeds(0,0,0));
//            }
//
//            if(hasDriven) {
//                if(stateTimer.seconds() > 2.0) {
//                    nextState(StateType.SLIDES, new Slide_Position(90));
//                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.TELEOP_POS, 0.1));
//                    nextState(StateType.DRIVE, new FarGrabToPlace());
//                } else if(stateTimer.seconds() > 1.0 && !hasSpit) {
//                    nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.STACK, -0.7));
//                    hasSpit = true;
//                }
//            }
//        }
//    }
//
//    private class FarGrabToPlace extends Executive.DrivingStateBase<Autonomous> {
//        boolean bool, bool2, bool3, bool4, bool6;
//        @Override
//        public void init(Executive.StateMachine<Autonomous> stateMachine) {
//            super.init(stateMachine);
//
//            ChassisSpeeds currentSpeeds = speedsSupplier.get();
//            PathPlannerPath path;
//
//            if(spikePosition.equals(SpikePosition.LEFT))
//                path = PathPlannerPath.fromPathFile("Blue Right Place spike 1");
//            else if (spikePosition.equals(SpikePosition.RIGHT))
//                path = PathPlannerPath.fromPathFile("Copy of Copy of Blue Right Place spike 3");
//            else
//                path = PathPlannerPath.fromPathFile("Copy of Blue Right Place spike 2");
//
//            if(allianceColor.equals(AllianceColor.RED)) {
//                path = path.flipPath();
//            }
//
//            trajectory = new PathPlannerTrajectory(path, currentSpeeds, poseSupplier.get().getRotation());
//
//            controller.reset(poseSupplier.get(), currentSpeeds);
//            nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.STACK, 0.0, 0.5));
//        }
//
//        @Override
//        public void update() {
//            super.update();
//
//            double currentTime = drivingTimer.seconds();
//            PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
//            Pose2d currentPose = poseSupplier.get();
//            ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
//            output.accept(targetSpeeds);
//
//            if(drivingTimer.seconds() > finishTime + 0.5 && !bool) {
//                nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.DRIVE, 0.0, 0.0));
//                stateMachine.changeState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.DOWN));
//                bool = true;
//            } else if(drivingTimer.seconds() > finishTime + 1.0 && !bool2 && bool) {
//                stateMachine.changeState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.GRAB, RobotConstants.ClawOrder.BOTH));
//                bool2 = true;
//            } else if(drivingTimer.seconds() > finishTime + 1.2 && !bool3 && bool2) {
////                    stateMachine.changeState(Executive.StateMachine.StateType.SLIDES, new Slide_Speed(-0.8));
//                bool3 = true;
//            } else if(drivingTimer.seconds() > finishTime + 1.5 && !bool4 && bool3) {
//                stateMachine.changeState(Executive.StateMachine.StateType.SLIDES, new Slide_Position(160));
//                stateMachine.changeState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.TELEOP_POS));
//                stateMachine.changeState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.RIGHT_HORIZONTAL, 0.8));
//                bool4 = true;
//            } else if(drivingTimer.seconds() > finishTime + 2.0 && !bool6 && bool4) {
//                stateMachine.changeState(Executive.StateMachine.StateType.SLIDES, new Slide_Position(60));
//                stateMachine.changeState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.BACK_BOARD));
//                bool6 = true;
//
//                stateTimer.reset();
//            }
//
//            if (currentTime > finishTime) {
//                output.accept(new ChassisSpeeds(0,0,0));
//            }
//
//            if(bool6 && stateTimer.seconds() > 1.0) {
//                nextState(StateType.DRIVE, new Stop());
//            }
//        }
//    }

    private class FarSpikeToBackBoard extends Executive.DrivingStateBase<Autonomous> {
        boolean hasSetArm = false, hasDriven = false, hasSetClaw = false;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;

            if(spikePosition.equals(SpikePosition.LEFT))
                path = PathPlannerPath.fromPathFile("skip cycle 1");
            else if (spikePosition.equals(SpikePosition.RIGHT))
                path = PathPlannerPath.fromPathFile("skip cycle 3");
            else
                path = PathPlannerPath.fromPathFile("skip cycle 2");

            if(allianceColor.equals(AllianceColor.RED)) {
                path = path.flipPath();
            }

            trajectory = new PathPlannerTrajectory(path, currentSpeeds, poseSupplier.get().getRotation());

            controller.reset(poseSupplier.get(), currentSpeeds);

            nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.BACK_BOARD));
        }

        @Override
        public void update() {
            super.update();

            double currentTime = drivingTimer.seconds();
            PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
            Pose2d currentPose = poseSupplier.get();
            ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
            output.accept(targetSpeeds);

            if(currentTime > finishTime - 1.0 && !hasSetArm) {
                nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.BACK_BOARD));
                nextState(StateType.SLIDES, new Slide_Position(100));
                hasSetArm = true;
            }

            if (currentTime > finishTime && !hasDriven) {
                hasDriven = true;
                stateTimer.reset();
                output.accept(new ChassisSpeeds(0,0,0));
            }

            if(hasDriven) {
                if(!hasSetClaw && stateTimer.seconds() > 0.3) {
                    nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.OPEN, RobotConstants.ClawOrder.LEFT, 0.1));
                    hasSetClaw = true;
                }

                if(hasSetClaw && stateTimer.seconds() > 0.6) {
                    nextState(StateType.SLIDES, new Slide_Position(80));
                    nextState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.VERTICAL, 0.15));
                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.TELEOP_POS, 0.1));
                    nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.DRIVE, 0.0));
                    nextState(StateType.DRIVE, new FarBackBoardToGrab());
                }
            }
        }
    }

    // State to grab from the center
    private class FarBackBoardToGrab extends Executive.DrivingStateBase<Autonomous> {
        boolean hasDriven = false, hasSetIntake = false, hasSpit = false;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;

            if(spikePosition.equals(SpikePosition.LEFT))
                path = PathPlannerPath.fromPathFile("Right Blue 1 center stack grab");
            else if (spikePosition.equals(SpikePosition.RIGHT))
                path = PathPlannerPath.fromPathFile("Copy of Right Blue 3 center stack grab");
            else
                path = PathPlannerPath.fromPathFile("Copy of Copy of Right Blue 3 center stack grab");

            if(allianceColor.equals(AllianceColor.RED)) {
                path = path.flipPath();
            }

            trajectory = new PathPlannerTrajectory(path, currentSpeeds, poseSupplier.get().getRotation());

            controller.reset(poseSupplier.get(), currentSpeeds);

            nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.BACK_BOARD));
        }

        @Override
        public void update() {
            super.update();

            double currentTime = drivingTimer.seconds();
            PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
            Pose2d currentPose = poseSupplier.get();
            ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
            output.accept(targetSpeeds);

            if(currentTime > finishTime - 2.0 && !hasSetIntake) {
                hasSetIntake = true;
                nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.STACK, 1.0, 0.0));
            }

            if (currentTime > finishTime && !hasDriven) {
                hasDriven = true;
                stateTimer.reset();
                output.accept(new ChassisSpeeds(0,0,0));
            }

            if(hasDriven) {
                if(stateTimer.seconds() > 1.0) {
                    nextState(StateType.SLIDES, new Slide_Position(90));
//                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.TELEOP_POS, 0.1));
                    nextState(StateType.DRIVE, new FarGrabToPlace());
                } else if(stateTimer.seconds() > 0.5 && !hasSpit) {
                    nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.STACK, -0.7));
                    hasSpit = true;
                }
            }
        }
    }

    private class FarGrabToPlace extends Executive.DrivingStateBase<Autonomous> {
        boolean hasSetArm = false, bool, bool2, bool3, bool4, bool6;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path = PathPlannerPath.fromPathFile("center to place");

            if(allianceColor.equals(AllianceColor.RED)) {
                path = path.flipPath();
            }

            trajectory = new PathPlannerTrajectory(path, currentSpeeds, poseSupplier.get().getRotation());

            controller.reset(poseSupplier.get(), currentSpeeds);
            nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.STACK, 0.0, 0.5));
            nextState(StateType.SLIDES, new Slide_Position(60));
        }

        @Override
        public void update() {
            super.update();

            double currentTime = drivingTimer.seconds();
            PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
            Pose2d currentPose = poseSupplier.get();
            ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
            output.accept(targetSpeeds);

            if(drivingTimer.seconds() > finishTime - 2.7 && !hasSetArm) {
                nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.TELEOP_POS));
                nextState(StateType.SLIDES, new Slide_Position(20));
                hasSetArm = true;
            }

            if(drivingTimer.seconds() > finishTime - 2.2 && !bool) {
                nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.DRIVE, 0.0, 0.0));
                stateMachine.changeState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.DOWN));
                bool = true;
            } else if(drivingTimer.seconds() > finishTime - 1.8 && !bool2 && bool) {
                stateMachine.changeState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.GRAB, RobotConstants.ClawOrder.BOTH));
                bool2 = true;
            } else if(drivingTimer.seconds() > finishTime - 1.5 && !bool3 && bool2) {
//                    stateMachine.changeState(Executive.StateMachine.StateType.SLIDES, new Slide_Speed(-0.8));
                bool3 = true;
            } else if(drivingTimer.seconds() > finishTime - 1.2 && !bool4 && bool3) {
                stateMachine.changeState(Executive.StateMachine.StateType.SLIDES, new Slide_Position(160));
                stateMachine.changeState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.TELEOP_POS));
                stateMachine.changeState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.RIGHT_HORIZONTAL, 0.8));
                bool4 = true;
            } else if(drivingTimer.seconds() > finishTime - 0.7 && !bool6 && bool4) {
                stateMachine.changeState(Executive.StateMachine.StateType.SLIDES, new Slide_Position(60));
                stateMachine.changeState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.BACK_BOARD));
                bool6 = true;

                stateTimer.reset();
            }

            if (currentTime > finishTime) {
                output.accept(new ChassisSpeeds(0,0,0));
            }

            if(bool6 && stateTimer.seconds() > 1.0) {
                nextState(StateType.DRIVE, new Stop());
            }
        }
    }

    static class Slide_Speed extends Executive.StateBase<Autonomous> {
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

    public static class Slide_Position extends Executive.StateBase<Autonomous> {
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

    public static class Claw_Position extends Executive.StateBase<Autonomous> {
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

    public static class Arm_Position extends Executive.StateBase<Autonomous> {
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

    public static class Wrist_Position extends Executive.StateBase<Autonomous> {
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
    public static class Intake_Position extends Executive.StateBase<Autonomous> {
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
}
package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.ARM_LEFT;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.ARM_RIGHT;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.CLAW_LEFT;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.CLAW_RIGHT;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.INTAKE;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.RAMP;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.WRIST;
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
    public static double visionTime = 1.5, spikePlace = 1.0;
    public static SpikePosition spikePosition = SpikePosition.NONE;

    public static RobotConstants.AutonomousRoutine routine = RobotConstants.AutonomousRoutine.SPIKE_PARK;
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
                new PIDConstants(3.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0),
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
//        @Override
//        public void init(Executive.StateMachine<Autonomous> stateMachine) {
//            super.init(stateMachine);
//
//            switch (startPosition) {
//                case AUDIENCE:
////                    setStartPosition(new Pose2d());
//                    break;
//                case BACK_BOARD:
//                    switch (routine) {
//                        case PARK:
////                            setStartPosition(
////                                    new Pose2d()
////                            );
//                            break;
//                        case SPIKE_PARK:
//                            setStartPosition(
//                                    trajectories[RobotConstants.AutonomousRoutine.SPIKE_PARK.ordinal()]
//                                            .getInitialState().getTargetHolonomicPose()
//                            );
//                            break;
//                        case SPIKE_PLACE_PARK:
//                            setStartPosition(
//                                    trajectories[RobotConstants.AutonomousRoutine.SPIKE_PLACE_PARK.ordinal()]
//                                            .getInitialState().getTargetHolonomicPose()
//                            );
//                            break;
//                        default:
//
//                    }
//                    break;
//                default:
//                   throw new IllegalArgumentException("Invalid start position");
//            }
//        }

        @Override
        public void update() {
            super.update();
            if(startPosition.equals(StartPosition.AUDIENCE)) {
                nextState(StateType.DRIVE, new Detect_Spike());
            } else {
                if(routine.equals(RobotConstants.AutonomousRoutine.PARK))
                    nextState(StateType.DRIVE, new Back_Park());
                else {
//                    spikePosition = SpikePosition.MIDDLE;
                    nextState(StateType.DRIVE, new Detect_Spike());
                }
            }
        }

        private void setStartPosition(Pose2d pose) {
            opMode.swerveDrive.resetOdometry(pose);
        }
    }

    private class Detect_Spike extends Executive.StateBase<Autonomous> {

        Webcam webcam = RobotConfiguration.WEBCAM.getAsWebcam();

        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);
            nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.HOLD));
            nextState(StateType.SLIDES, new Slide_Position(20));
            nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.GRAB, RobotConstants.ClawOrder.BOTH));
            nextState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.VERTICAL));
        }

        @Override
        public void update() {
            super.update();
            Rect rect = allianceColor.equals(AllianceColor.BLUE)
                    ? webcam.getProcessor().getBlueRect()
                    : webcam.getProcessor().getRedRect();

            try {
                double x = rect.x + rect.width / 2.0;
                if (x < leftBound)
                    spikePosition = SpikePosition.LEFT;
                else if (x > rightBound)
                    spikePosition = SpikePosition.RIGHT;
                else
                    spikePosition = SpikePosition.MIDDLE;
            } catch (NullPointerException ignore) {
            }

            opMode.telemetry.addData("Spike", spikePosition.name());

            if (stateTimer.seconds() > visionTime) {
                webcam.stop();
                nextState(StateType.DRIVE, new CompSpike());
            }
        }
    }

    private class CompSpike extends Executive.StateBase<Autonomous> {
        private boolean hasRun = false, hasDriven = false, hasSetArm = false, hasSetClaw = false, bool = false;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;
            if(allianceColor.equals(AllianceColor.BLUE)) {
                if(spikePosition.equals(SpikePosition.LEFT))
                    path = PathPlannerPath.fromPathFile("TopStartToSpikeLeft");
                else if (spikePosition.equals(SpikePosition.RIGHT))
                    path = PathPlannerPath.fromPathFile("TopStartToSpikeRight");
                else
                    path = PathPlannerPath.fromPathFile("TopStartToSpikeMiddle");
                //ToDo replace with start position
                opMode.swerveDrive.resetOdometry(new Pose2d(new Translation2d(0.215, 2.127), Rotation2d.fromDegrees(180.0)));
            } else {
                if(spikePosition.equals(SpikePosition.LEFT))
                    path = PathPlannerPath.fromPathFile("RedTopStartToSpikeLeft");
                else if (spikePosition.equals(SpikePosition.RIGHT))
                    path = PathPlannerPath.fromPathFile("RedTopStartToSpikeRight");
                else
                    path = PathPlannerPath.fromPathFile("RedTopStartToSpikeMiddle");
                opMode.swerveDrive.resetOdometry(new Pose2d(new Translation2d(3.40, 2.13), Rotation2d.fromDegrees(0.0)));
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

                if(!hasSetArm && stateTimer.seconds() > 0.2) {
                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.BACK_BOARD));
                    nextState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.RIGHT_HORIZONTAL));
                    hasSetArm = true;
                }

                if(!bool && stateTimer.seconds() > 0.6) {
                    bool = true;
                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.SPIKE));
                }

                if (currentTime > trajectory.getTotalTimeSeconds()+ 0.25) {
                    hasDriven = true;
                    stateTimer.reset();
                    output.accept(new ChassisSpeeds(0,0,0));
                }
            } else {
                if(hasSetArm) {
                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.SPIKE));
                    hasSetArm = false;
                }
                if(!hasSetClaw && stateTimer.seconds() > 0.2) {
                    nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.OPEN, RobotConstants.ClawOrder.RIGHT));
                    hasSetClaw = true;
                }

                if(hasSetClaw && stateTimer.seconds() > 2.0) {
                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.HOLD));
                    nextState(StateType.SLIDES, new Slide_Position(100));
//                    nextState(StateType.DRIVE, new CompPlace());
                    nextState(StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.VERTICAL));
                }
            }
        }
    }

    private class CompPlace extends Executive.StateBase<Autonomous> {
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
                    path = PathPlannerPath.fromPathFile("TopSpikeRightToPlaceRight");
                else
                    path = PathPlannerPath.fromPathFile("TopSpikeMiddleToPlaceMiddle");
            } else {
                if(spikePosition.equals(SpikePosition.LEFT))
                    path = PathPlannerPath.fromPathFile("RedTopSpikeLeftToPlaceLeft");
                else if (spikePosition.equals(SpikePosition.RIGHT))
                    path = PathPlannerPath.fromPathFile("RedTopSpikeRightToPlaceRight");
                else
                    path = PathPlannerPath.fromPathFile("RedTopSpikeMiddleToPlaceMiddle");
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
                if(!hasSetClaw && stateTimer.seconds() > 1.0) {
                    nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.OPEN, RobotConstants.ClawOrder.LEFT));
                    hasSetClaw = true;
                }

                if(stateTimer.seconds() > 2.0) {
                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.HOLD, 0.3));
                    nextState(StateType.DRIVE, new CompPark());
                }
            }
        }
    }

    private class CompPark extends Executive.StateBase<Autonomous> {
        
        private boolean hasRun = false, hasDriven = false;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;
            if(allianceColor.equals(AllianceColor.BLUE)) {
                if(spikePosition.equals(SpikePosition.LEFT))
                    path = PathPlannerPath.fromPathFile("TopPlaceLeftToPark");
                else if (spikePosition.equals(SpikePosition.RIGHT))
                    path = PathPlannerPath.fromPathFile("TopPlaceRightToPark");
                else
                    path = PathPlannerPath.fromPathFile("TopPlaceMiddleToPark");
            } else {
                if(spikePosition.equals(SpikePosition.LEFT))
                    path = PathPlannerPath.fromPathFile("RedTopPlaceLeftToPark");
                else if (spikePosition.equals(SpikePosition.RIGHT))
                    path = PathPlannerPath.fromPathFile("RedTopPlaceRightToPark");
                else
                    path = PathPlannerPath.fromPathFile("RedTopPlaceMiddleToPark");
            }

            trajectory = new PathPlannerTrajectory(path, currentSpeeds);

            controller.reset(poseSupplier.get(), currentSpeeds);

            nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.HOLD));
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
                if(stateTimer.seconds() > 1.0) {
                    nextState(StateType.DRIVE, new Stop());
                }
            }
        }
    }

    private class Place extends Executive.StateBase<Autonomous> {
        
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
                //ToDo replace with start position
                opMode.swerveDrive.resetOdometry(new Pose2d(new Translation2d(0.215, 2.127), Rotation2d.fromDegrees(180.0)));
            } else {
                path = PathPlannerPath.fromPathFile("RedPlace");
                opMode.swerveDrive.resetOdometry(new Pose2d(new Translation2d(3.20, 2.14), Rotation2d.fromDegrees(0.0)));
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

                if(!hasSetArm && stateTimer.seconds() > 0.6) {
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

                if(stateTimer.seconds() > 2.0) {
                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.HOLD, 0.3));
                    nextState(StateType.DRIVE, new Spike());
                }
            }
        }
    }

    private class Spike extends Executive.StateBase<Autonomous> {
        
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
                path = PathPlannerPath.fromPathFile("RedPlace");
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

                if (currentTime > trajectory.getTotalTimeSeconds()+1.0) {
                    hasDriven = true;
                    stateTimer.reset();
                    output.accept(new ChassisSpeeds(0,0,0));
                }
            } else {
                if(!hasSetArm && stateTimer.seconds() > 1.0) {
                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.SPIKE));
                    hasSetArm = true;
                } else if(!hasSetClaw && stateTimer.seconds() > 1.5 && hasSetArm) {
                    nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.OPEN, RobotConstants.ClawOrder.RIGHT));
                    hasSetClaw = true;
                }

                if(hasSetClaw && stateTimer.seconds() > 2.0) {
                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.HOLD));
//                    nextState(StateType.DRIVE, new Stop());
                }
            }
        }
    }

    private class PlaceToPark extends Executive.StateBase<Autonomous> {
        
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;
            if(allianceColor.equals(AllianceColor.BLUE))
                path = PathPlannerPath.fromPathFile("BluePlacePark");
            else
                path = PathPlannerPath.fromPathFile("RedPlacePark");

            trajectory = new PathPlannerTrajectory(path, currentSpeeds);

            controller.reset(poseSupplier.get(), currentSpeeds);
        }

        @Override
        public void update() {
            super.update();
            double currentTime = stateTimer.seconds();
            PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
            Pose2d currentPose = poseSupplier.get();
            ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
            output.accept(targetSpeeds);

            if (currentTime > trajectory.getTotalTimeSeconds()) {
                nextState(StateType.DRIVE, new Stop());
            }
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
            activeProfile = MotionProfileGenerator.generateSimpleMotionProfile(start, goal, 1000, 600);
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
                    activeProfile = MotionProfileGenerator.generateSimpleMotionProfile(start, goal, 1000, 600);
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
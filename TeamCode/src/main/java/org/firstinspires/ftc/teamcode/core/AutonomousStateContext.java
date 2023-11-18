package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.ARM_LEFT;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.ARM_RIGHT;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.CLAW_LEFT;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.CLAW_RIGHT;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.INTAKE;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.RAMP;
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
    public static double visionTime = 4.0, spikePlace = 1.0;
    public SpikePosition spikePosition = SpikePosition.NONE;

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
                0.013,
                1.8,
                0.2514
        );

        poseSupplier = autonomous.swerveDrive::getPose;
        speedsSupplier = autonomous.swerveDrive::getRobotVelocity;
        output = autonomous.swerveDrive::drive;

        loadPathPlannerTrajectories();

        stateMachine.changeState(StateType.DRIVE, new Start());
        stateMachine.changeState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.START));
        stateMachine.changeState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.START, 0.0));
    }

    @Override
    public void update() {
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
                nextState(StateType.DRIVE, new Audience_Park());
            } else {
                if(routine.equals(RobotConstants.AutonomousRoutine.PARK))
                    nextState(StateType.DRIVE, new Back_Park());
                else {
                    nextState(StateType.DRIVE, new Place());
                }
            }
        }

        private void setStartPosition(Pose2d pose) {
            opMode.swerveDrive.resetOdometry(pose);
        }
    }

    private class Place extends Executive.StateBase<Autonomous> {
        private PathPlannerTrajectory generatedTrajectory;
        private boolean hasRun = false;
        private boolean hasDriven = false;
        private boolean hasSetArm = false;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;
            if(allianceColor.equals(AllianceColor.BLUE)) {
                path = PathPlannerPath.fromPathFile("BluePlace");
                opMode.swerveDrive.resetOdometry(new Pose2d(new Translation2d(0.23, 2.13), Rotation2d.fromDegrees(180.0)));
            } else {
                path = PathPlannerPath.fromPathFile("RedPlace");
                opMode.swerveDrive.resetOdometry(new Pose2d(new Translation2d(3.20, 2.14), Rotation2d.fromDegrees(0.0)));
            }

            generatedTrajectory = new PathPlannerTrajectory(path, currentSpeeds);

            controller.reset(poseSupplier.get(), currentSpeeds);

            nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.DRIVE, 0.0));
            nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.HOLD));
            nextState(StateType.SLIDES, new Slide_Position(100));
            nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.CLOSE, RobotConstants.ClawOrder.BOTH));
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
                PathPlannerTrajectory.State targetState = generatedTrajectory.sample(currentTime);
                Pose2d currentPose = poseSupplier.get();
                ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
                output.accept(targetSpeeds);

                if(!hasSetArm && stateTimer.seconds() > 0.8) {
                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.BACK_BOARD));
                    hasSetArm = true;
                }

                if (currentTime > generatedTrajectory.getTotalTimeSeconds()) {
                    hasDriven = true;
                    stateTimer.reset();
                    output.accept(new ChassisSpeeds(0,0,0));
                }
            } else {
                if(stateTimer.seconds() > 1.0) {
                    nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.OPEN, RobotConstants.ClawOrder.BOTH));
                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.HOLD, 0.1));
                    nextState(StateType.DRIVE, new PlaceToPark());
                }
            }
        }
    }

    private class PlaceToPark extends Executive.StateBase<Autonomous> {
        private PathPlannerTrajectory generatedTrajectory;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;
            if(allianceColor.equals(AllianceColor.BLUE))
                path = PathPlannerPath.fromPathFile("BluePlacePark");
            else
                path = PathPlannerPath.fromPathFile("RedPlacePark");

            generatedTrajectory = new PathPlannerTrajectory(path, currentSpeeds);

            controller.reset(poseSupplier.get(), currentSpeeds);
        }

        @Override
        public void update() {
            super.update();
            double currentTime = stateTimer.seconds();
            PathPlannerTrajectory.State targetState = generatedTrajectory.sample(currentTime);
            Pose2d currentPose = poseSupplier.get();
            ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
            output.accept(targetSpeeds);

            if (currentTime > generatedTrajectory.getTotalTimeSeconds()) {
                nextState(StateType.DRIVE, new Stop());
            }
        }
    }

//    private class Detect_Spike extends Executive.StateBase<Autonomous> {
//
//        Webcam webcam = RobotConfiguration.WEBCAM.getAsWebcam();
//
//        @Override
//        public void init(Executive.StateMachine<Autonomous> stateMachine) {
//            super.init(stateMachine);
//            nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.DRIVE, 0.0));
//            nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.HOLD));
//            nextState(StateType.SLIDES, new Slide_Position(100));
//            nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.CLOSE, RobotConstants.ClawOrder.BOTH));
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            Rect rect = allianceColor.equals(AllianceColor.BLUE)
//                    ? webcam.getProcessor().getBlueRect()
//                    : webcam.getProcessor().getRedRect();
//
//            try {
//                double x = rect.x + rect.width/2.0;
//                if(x < leftBound)
//                    spikePosition = SpikePosition.LEFT;
//                else if(x > rightBound)
//                    spikePosition = SpikePosition.RIGHT;
//                else
//                    spikePosition = SpikePosition.MIDDLE;
//            } catch (NullPointerException ignore) {}
//
//            opMode.telemetry.addData("Spike", spikePosition.name());
//
//            if(stateTimer.seconds() > visionTime) {
//                webcam.stop();
//                nextState(StateType.DRIVE, new Spike());
//            }
//        }
//    }
//
//    private class Spike extends Executive.StateBase<Autonomous> {
//        private PathPlannerTrajectory generatedTrajectory;
//        private boolean hasRun = false;
//        private boolean hasDriven = false;
//        @Override
//        public void init(Executive.StateMachine<Autonomous> stateMachine) {
//            super.init(stateMachine);
//
//            ChassisSpeeds currentSpeeds = speedsSupplier.get();
//            PathPlannerPath path = PathPlannerPath.fromPathFile("MiddleSpike");;
////            if(spikePosition.equals(SpikePosition.RIGHT)) {
////                path = PathPlannerPath.fromPathFile("RightSpike");
////            } else if (spikePosition.equals(SpikePosition.LEFT)) {
////                path = PathPlannerPath.fromPathFile("LeftSpike");
////            } else {
////                path = PathPlannerPath.fromPathFile("MiddleSpike");
////            }
//
//            generatedTrajectory = new PathPlannerTrajectory(path, currentSpeeds);
//
//            opMode.swerveDrive.resetOdometry(new Pose2d(new Translation2d(0.23, 2.13), Rotation2d.fromDegrees(180.0)));
//
//            controller.reset(poseSupplier.get(), currentSpeeds);
//
////            nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.INTAKE, 0.0));
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            if(!hasDriven) {
//                if (!hasRun) {
//                    stateTimer.reset();
//                    hasRun = true;
//                }
//                double currentTime = stateTimer.seconds();
//                PathPlannerTrajectory.State targetState = generatedTrajectory.sample(currentTime);
//                output.accept(controller.calculateRobotRelativeSpeeds(poseSupplier.get(), targetState));
//
//                if (currentTime > generatedTrajectory.getTotalTimeSeconds()) {
//                    hasDriven = true;
//                    stateTimer.reset();
////                    nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.INTAKE, -0.4, 0.0));
//                }
//            } else {
//                if(stateTimer.seconds() > 1.0) {
////                    nextState(StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.DRIVE, 0.0, 0.0));
//                    nextState(StateType.DRIVE, new Spike_To_Place());
//                }
//            }
//        }
//    }
//
//    private class Spike_To_Place extends Executive.StateBase<Autonomous> {
//        private PathPlannerTrajectory generatedTrajectory;
//        private boolean hasRun = false;
//        private boolean hasDriven = false;
//        @Override
//        public void init(Executive.StateMachine<Autonomous> stateMachine) {
//            super.init(stateMachine);
//
//            ChassisSpeeds currentSpeeds = speedsSupplier.get();
//            PathPlannerPath path = null;
//            if(spikePosition.equals(SpikePosition.RIGHT)) {
//                path = PathPlannerPath.fromPathFile("RightSpikePlace");
//            } else if (spikePosition.equals(SpikePosition.LEFT)) {
//                path = PathPlannerPath.fromPathFile("LeftSpikePlace");
//            } else {
//                path = PathPlannerPath.fromPathFile("MiddleSpikePlace");
//            }
//
//            generatedTrajectory = new PathPlannerTrajectory(path, currentSpeeds);
//
//            controller.reset(poseSupplier.get(), currentSpeeds);
//            nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.BACK_BOARD));
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            if(!hasDriven) {
//                if (!hasRun) {
//                    stateTimer.reset();
//                    hasRun = true;
//                }
//                double currentTime = stateTimer.seconds();
//                PathPlannerTrajectory.State targetState = generatedTrajectory.sample(currentTime);
//                Pose2d currentPose = poseSupplier.get();
//                ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
//                output.accept(targetSpeeds);
//
//                if (currentTime > generatedTrajectory.getTotalTimeSeconds()) {
//                    hasDriven = true;
//                    stateTimer.reset();
//                }
//            } else {
//                if(stateTimer.seconds() > 1.0) {
//                    nextState(StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.OPEN, RobotConstants.ClawOrder.BOTH));
//                    nextState(StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.HOLD));
//                    nextState(StateType.DRIVE, new Place_To_Park());
//                }
//            }
//        }
//    }
//
//    private class Place_To_Park extends Executive.StateBase<Autonomous> {
//        private PathPlannerTrajectory generatedTrajectory;
//        private boolean hasRun = false;
//        @Override
//        public void init(Executive.StateMachine<Autonomous> stateMachine) {
//            super.init(stateMachine);
//
//            ChassisSpeeds currentSpeeds = speedsSupplier.get();
//
//            PathPlannerPath path = PathPlannerPath.fromPathFile("PlacePark");
//            generatedTrajectory = new PathPlannerTrajectory(path, currentSpeeds);
//
//            opMode.swerveDrive.resetOdometry(generatedTrajectory.getInitialTargetHolonomicPose());
//            controller.reset(poseSupplier.get(), currentSpeeds);
//        }
//
//        @Override
//        public void update() {
//            super.update();
//            if(!hasRun) {
//                stateTimer.reset();
//                hasRun = true;
//            }
//            double currentTime = stateTimer.seconds();
//            PathPlannerTrajectory.State targetState = generatedTrajectory.sample(currentTime);
//            Pose2d currentPose = poseSupplier.get();
//            ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
//            output.accept(targetSpeeds);
//
//            if (currentTime > generatedTrajectory.getTotalTimeSeconds())
//                nextState(StateType.DRIVE, new Stop());
//        }
//    }

    private class Audience_Park extends Executive.StateBase<Autonomous> {
        private PathPlannerTrajectory generatedTrajectory;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();
            PathPlannerPath path;
            if(allianceColor.equals(AllianceColor.BLUE)) {
                path = PathPlannerPath.fromPathFile("BlueAudiencePark");
                opMode.swerveDrive.resetOdometry(new Pose2d(new Translation2d(0.23, 0.91), Rotation2d.fromDegrees(180.0)));
            } else {
                path = PathPlannerPath.fromPathFile("RedAudiencePark");
                opMode.swerveDrive.resetOdometry(new Pose2d(new Translation2d(3.20, 0.91), Rotation2d.fromDegrees(0.0)));
            }
            generatedTrajectory = new PathPlannerTrajectory(path, currentSpeeds);

            controller.reset(opMode.swerveDrive.getPose(), new ChassisSpeeds(0.0, 0.0, 0.0));
            Pose2d currentPose = poseSupplier.get();

            controller.reset(currentPose, currentSpeeds);
        }

        @Override
        public void update() {
            super.update();
            double currentTime = stateTimer.seconds();
            PathPlannerTrajectory.State targetState = generatedTrajectory.sample(currentTime);
            Pose2d currentPose = poseSupplier.get();
            output.accept(controller.calculateRobotRelativeSpeeds(currentPose, targetState));

            if (currentTime > generatedTrajectory.getTotalTimeSeconds())
                nextState(StateType.DRIVE, new Stop());
        }
    }

    private class Back_Park extends Executive.StateBase<Autonomous> {
        private PathPlannerTrajectory generatedTrajectory;
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
            generatedTrajectory = new PathPlannerTrajectory(path, currentSpeeds);

            controller.reset(opMode.swerveDrive.getPose(), new ChassisSpeeds(0.0, 0.0, 0.0));
            Pose2d currentPose = poseSupplier.get();

            controller.reset(currentPose, currentSpeeds);
        }

        @Override
        public void update() {
            super.update();
            double currentTime = stateTimer.seconds();
            PathPlannerTrajectory.State targetState = generatedTrajectory.sample(currentTime);
            Pose2d currentPose = poseSupplier.get();
            output.accept(controller.calculateRobotRelativeSpeeds(currentPose, targetState));

            if (currentTime > generatedTrajectory.getTotalTimeSeconds())
                nextState(StateType.DRIVE, new Stop());
        }
    }

    private static class Slide_Position extends Executive.StateBase<Autonomous> {

        private final Motor left = RobotConfiguration.SLIDE_LEFT.getAsMotor(), right = RobotConfiguration.SLIDE_RIGHT.getAsMotor();
        private final Encoder slides = RobotConfiguration.LIFT_ENCODER.getAsEncoder();
        private final double setpoint;
        private MotionProfile activeProfile;
        private double profileStart;
        private final PIDFController controller = RobotConstants.slideController;


        Slide_Position(double setpoint) {
            this.setpoint = setpoint;
        }

        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);
            profileStart = stateTimer.seconds();
            MotionState start = new MotionState(slides.getCurrentPosition(), 0, 0, 0);
            MotionState goal = new MotionState(setpoint, 0, 0, 0);
            activeProfile = MotionProfileGenerator.generateSimpleMotionProfile(start, goal, 1000, 600);
        }

        @Override
        public void update() {
            super.update();

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
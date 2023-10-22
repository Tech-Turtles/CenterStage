package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Servo;
import org.firstinspires.ftc.teamcode.opmode.autonomous.Autonomous;
import org.firstinspires.ftc.teamcode.utility.autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.utility.autonomous.Executive;
import org.firstinspires.ftc.teamcode.utility.autonomous.StartPosition;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utility.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.utility.pathplanner.controllers.PPHolonomicDriveController;
import org.firstinspires.ftc.teamcode.utility.pathplanner.path.PathPlannerPath;
import org.firstinspires.ftc.teamcode.utility.pathplanner.path.PathPlannerTrajectory;
import org.firstinspires.ftc.teamcode.utility.pathplanner.util.PIDConstants;

import java.util.function.Consumer;
import java.util.function.Supplier;

import static org.firstinspires.ftc.teamcode.utility.autonomous.Executive.StateMachine.StateType;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.*;

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

    public static RobotConstants.AutonomousRoutine routine = RobotConstants.AutonomousRoutine.PARK;
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
                new PIDConstants(1.0, 0.0, 0.0),
                new PIDConstants(2.0, 0.0, 0.0),
                0.013,
                2.2,
                0.2032
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

            switch (startPosition) {
                case AUDIENCE:
                    setStartPosition(new Pose2d());
                    break;
                case BACK_BOARD:
                    switch (routine) {
                        case PARK:
//                            setStartPosition(
//                                    new Pose2d()
//                            );
                            break;
                        case SPIKE_PARK:
                            setStartPosition(
                                    trajectories[RobotConstants.AutonomousRoutine.SPIKE_PARK.ordinal()]
                                            .getInitialState().getTargetHolonomicPose()
                            );
                            break;
                        case SPIKE_PLACE_PARK:
                            setStartPosition(
                                    trajectories[RobotConstants.AutonomousRoutine.SPIKE_PLACE_PARK.ordinal()]
                                            .getInitialState().getTargetHolonomicPose()
                            );
                            break;
                        default:

                    }
                    break;
                default:
                   throw new IllegalArgumentException("Invalid start position");
            }
        }

        @Override
        public void update() {
            super.update();
            opMode.telemetry.addData("State Position", opMode.swerveDrive.getPose());
            if(opMode.primary.A()) {
                nextState(StateType.DRIVE, new Back_Park());
            }
        }

        private void setStartPosition(Pose2d pose) {
            opMode.swerveDrive.resetOdometry(pose);
        }
    }

    private class Back_Park extends Executive.StateBase<Autonomous> {
        private PathPlannerTrajectory generatedTrajectory;
        private boolean hasRun = false;
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            ChassisSpeeds currentSpeeds = speedsSupplier.get();

            PathPlannerPath path = PathPlannerPath.fromPathFile("Forward");

            generatedTrajectory = new PathPlannerTrajectory(path, currentSpeeds);

            opMode.swerveDrive.resetOdometry(generatedTrajectory.getInitialTargetHolonomicPose());

            controller.reset(opMode.swerveDrive.getPose(), new ChassisSpeeds(0.0, 0.0, 0.0));
            Pose2d currentPose = poseSupplier.get();

            controller.reset(currentPose, currentSpeeds);

            stateTimer.reset();
        }

        @Override
        public void update() {
            super.update();
            if(!hasRun) {
                stateTimer.reset();
                hasRun = true;
            }
            double currentTime = stateTimer.seconds();
            PathPlannerTrajectory.State targetState = generatedTrajectory.sample(currentTime);
            Pose2d currentPose = poseSupplier.get();
            opMode.telemetry.addData("Pose", currentPose);
            ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
            opMode.telemetry.addData("Speeds", targetSpeeds);
            output.accept(targetSpeeds);

            if(currentTime > generatedTrajectory.getTotalTimeSeconds())
                nextState(StateType.DRIVE, new Stop());
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

            if(stateTimer.seconds() < delay)
                return;

            RAMP.getAsServo().setPosition(pos);
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
package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.ARM_LEFT;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.ARM_RIGHT;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.CLAW_LEFT;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.CLAW_RIGHT;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.INTAKE;
import static org.firstinspires.ftc.teamcode.core.RobotConfiguration.RAMP;
import static org.firstinspires.ftc.teamcode.opmode.autonomous.SlidePIDTest.kG;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Encoder;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Servo;
import org.firstinspires.ftc.teamcode.opmode.teleop.Manual;
import org.firstinspires.ftc.teamcode.utility.autonomous.Executive;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Pose2d;

@TeleOp
@Config
public class TransferTest extends RobotHardware {
    private final Executive.StateMachine<TransferTest> stateMachine;

    public static double slideStart = 60;
    public static double slideDown = 20;

    public TransferTest() {
        stateMachine = new Executive.StateMachine<>(this);
        stateMachine.update();
    }

    @Override
    public void init() {
        super.init();
        stateMachine.init();
        stateMachine.changeState(Executive.StateMachine.StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.VERTICAL));
        stateMachine.changeState(Executive.StateMachine.StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.TELEOP_POS));
        stateMachine.changeState(Executive.StateMachine.StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.OPEN, RobotConstants.ClawOrder.BOTH));
        stateMachine.changeState(Executive.StateMachine.StateType.SLIDES, new Slide_Position(slideStart));
    }

    @Override
    public void init_loop() {
        super.init_loop();
        stateMachine.update();
        RobotConfiguration.RAMP.getAsServo().setPosition(RobotConstants.IntakePosition.DRIVE.getPosition());
    }

    @Override
    public void start() {
        super.start();
        stateMachine.changeState(Executive.StateMachine.StateType.DRIVE, new Transfer());
        stateMachine.changeState(Executive.StateMachine.StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.DOWN));
        stateMachine.changeState(Executive.StateMachine.StateType.SLIDES, new Slide_Position(slideDown));
        stateMachine.changeState(Executive.StateMachine.StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.VERTICAL));
        stateMachine.changeState(Executive.StateMachine.StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.DRIVE, 0.0));
    }

    @Override
    public void loop() {
        super.loop();
        stateMachine.update();
    }
    private class Transfer extends Executive.StateBase<TransferTest> {
        boolean hasSetArm = false, bool, bool2, bool3, bool4, bool6;
        double finishTime = 4;
        @Override
        public void update() {
            super.update();
            if(stateTimer.seconds() > finishTime - 2.2 && !bool) {
                nextState(Executive.StateMachine.StateType.INTAKE, new Intake_Position(RobotConstants.IntakePosition.DRIVE, 0.0, 0.0));
                stateMachine.changeState(Executive.StateMachine.StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.DOWN));
                bool = true;
            } else if(stateTimer.seconds() > finishTime - 1.8 && !bool2 && bool) {
                stateMachine.changeState(Executive.StateMachine.StateType.CLAW, new Claw_Position(RobotConstants.ClawPosition.GRAB, RobotConstants.ClawOrder.BOTH));
                bool2 = true;
            } else if(stateTimer.seconds() > finishTime - 1.5 && !bool3 && bool2) {
//                    stateMachine.changeState(Executive.StateMachine.StateType.SLIDES, new Slide_Speed(-0.8));
                bool3 = true;
            } else if(stateTimer.seconds() > finishTime - 1.2 && !bool4 && bool3) {
                stateMachine.changeState(Executive.StateMachine.StateType.SLIDES, new Slide_Position(160));
                stateMachine.changeState(Executive.StateMachine.StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.TELEOP_POS));
                stateMachine.changeState(Executive.StateMachine.StateType.WRIST, new Wrist_Position(RobotConstants.WristPosition.RIGHT_HORIZONTAL, 0.8));
                bool4 = true;
            } else if(stateTimer.seconds() > finishTime - 0.7 && !bool6 && bool4) {
                stateMachine.changeState(Executive.StateMachine.StateType.SLIDES, new Slide_Position(60));
                stateMachine.changeState(Executive.StateMachine.StateType.ARM, new Arm_Position(RobotConstants.ArmPosition.BACK_BOARD));
                bool6 = true;

                stateTimer.reset();
            }
        }
    }

    public static class Slide_Position extends Executive.StateBase<TransferTest> {
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
        public void init(Executive.StateMachine<TransferTest> stateMachine) {
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

    public static class Claw_Position extends Executive.StateBase<TransferTest> {
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

    public static class Arm_Position extends Executive.StateBase<TransferTest> {
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

    public static class Wrist_Position extends Executive.StateBase<TransferTest> {
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
    public static class Intake_Position extends Executive.StateBase<TransferTest> {
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
}

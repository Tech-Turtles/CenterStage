package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.core.RobotConstants.DEADZONE;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.IntakePosition;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.OUTTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.WRIST_CENTER;
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
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.utility.autonomous.Executive;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Translation2d;

@Config
@TeleOp(name="Manual", group="A")
public class Manual extends RobotHardware {
    public static double precisionMode = 1.0;
    private final double precisionPercentage = 0.4;
    public static boolean fieldRelative = true;
    public static boolean headingCorrection = true;
    private final Executive.StateMachine<Manual> stateMachine;

    private RobotConstants.ClawPosition left = RobotConstants.ClawPosition.OPEN, right = RobotConstants.ClawPosition.OPEN;
    private RobotConstants.ArmPosition armPosition = RobotConstants.ArmPosition.START;
    private RobotConstants.WristPosition wristPosition = RobotConstants.WristPosition.START;

    public static double slideDownSpeed = 0.6, slideSpeed = 1.0, liftSpeed = 1.0;

    public Manual() {
        stateMachine = new Executive.StateMachine<>(this);
        stateMachine.update();
    }

    @Override
    public void init() {
        super.init();
        stateMachine.init();
        armPosition = RobotConstants.ArmPosition.START;
        left = RobotConstants.ClawPosition.OPEN;
        right = RobotConstants.ClawPosition.OPEN;
        RobotConfiguration.DRONE.getAsServo().setPosition(1.0);
    }

    @Override
    public void init_loop() {
        super.init_loop();
        stateMachine.update();
        RobotConfiguration.CLAW_LEFT.getAsServo().setPosition(RobotConstants.ClawPosition.GRAB.getLeftPos());
        RobotConfiguration.CLAW_RIGHT.getAsServo().setPosition(RobotConstants.ClawPosition.GRAB.getRightPos());
        RobotConfiguration.RAMP.getAsServo().setPosition(IntakePosition.START.getPosition());
        RobotConfiguration.ARM_LEFT.getAsServo().setPosition(armPosition.getLeftPos());
        RobotConfiguration.ARM_RIGHT.getAsServo().setPosition(armPosition.getRightPos());
        RobotConfiguration.WRIST.getAsServo().setPosition(wristPosition.getPosition());
    }

    @Override
    public void start() {
        super.start();
        stateMachine.changeState(Executive.StateMachine.StateType.DRIVE, new Drive_Manual());
        stateMachine.changeState(Executive.StateMachine.StateType.SLIDES, new Slide_Manual());

        if(fieldRelative)
            primary.setLedColor(0.0, 0.0, 1.0, -1);
        else
            primary.setLedColor(1.0, 1.0, 0.0, -1);

        swerveDrive.zeroGyro();
        swerveDrive.resetOdometry(new Pose2d());
    }

    @Override
    public void loop() {
        super.loop();
        stateMachine.update();
    }
    
    class Drive_Manual extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();

            if (primary.AOnce()) {
//                swerveDrive.setMaximumSpeed(precisionMode ? SWERVE_MAX_SPEED : SWERVE_PRECISION_SPEED);
                precisionMode = precisionMode == 1.0 ? precisionPercentage : 1.0;
            }

            if (primary.YOnce()) {
                swerveDrive.zeroGyro();
                swerveDrive.resetOdometry(new Pose2d());
            }

            if (primary.BOnce()) {
                fieldRelative = !fieldRelative;
                if (fieldRelative)
                    primary.setLedColor(0.0, 0.0, 1.0, -1);
                else
                    primary.setLedColor(1.0, 1.0, 0.0, -1);
            }

            if (primary.XOnce())
                headingCorrection = !headingCorrection;

            if (primary.rightStickButtonOnce()) {
                SwerveDrive.lastHeadingRadians = (3.0 * Math.PI) / 2.0;
                SwerveDrive.updatedHeading = true;
            }

            double xV = -primary.left_stick_y * swerveControllerConfiguration.maxSpeed * precisionMode;
            double yV = -primary.left_stick_x * swerveControllerConfiguration.maxSpeed * precisionMode;
            double thetaV = -primary.right_stick_x * swerveControllerConfiguration.maxAngularVelocity * precisionMode;
            swerveDrive.drive(new Translation2d(xV, yV), thetaV, fieldRelative, true, headingCorrection);
            swerveDrive.updateOdometry();

            if (primary.right_trigger > DEADZONE) {
                RobotConfiguration.INTAKE.getAsMotor().setPower(INTAKE_SPEED * primary.right_trigger);
                RobotConfiguration.RAMP.getAsServo().setPosition(IntakePosition.INTAKE.getPosition());
            } else if (primary.left_trigger > DEADZONE) {
                RobotConfiguration.INTAKE.getAsMotor().setPower(OUTTAKE_SPEED * primary.left_trigger);
                RobotConfiguration.RAMP.getAsServo().setPosition(IntakePosition.INTAKE.getPosition());
            } else if (primary.dpadUp()) {
                RobotConfiguration.INTAKE.getAsMotor().setPower(INTAKE_SPEED);
                RobotConfiguration.RAMP.getAsServo().setPosition(IntakePosition.DRIVE.getPosition());
            } else if (primary.dpadDown()) {
                RobotConfiguration.INTAKE.getAsMotor().setPower(0.0);
                RobotConfiguration.RAMP.getAsServo().setPosition(IntakePosition.INTAKE.getPosition());
            } else {
                RobotConfiguration.INTAKE.getAsMotor().setPower(0.0);
                if(!stateMachine.getCurrentStateByType(Executive.StateMachine.StateType.INTAKE).equals(DroneLaunch.class))
                    RobotConfiguration.RAMP.getAsServo().setPosition(IntakePosition.DRIVE.getPosition());
            }

            if(primary.dpadLeftOnce())
                nextState(Executive.StateMachine.StateType.INTAKE, new DroneLaunch());

            if (secondary.leftTriggerOnce()) {
//                right = right.equals(RobotConstants.ClawPosition.OPEN) ? RobotConstants.ClawPosition.CLOSE : RobotConstants.ClawPosition.OPEN;
                right = RobotConstants.ClawPosition.OPEN;
            } else if (secondary.leftBumperOnce()) {
                right = RobotConstants.ClawPosition.GRAB;
            }

            if (secondary.rightTriggerOnce()) {
//                left = left.equals(RobotConstants.ClawPosition.OPEN) ? RobotConstants.ClawPosition.CLOSE : RobotConstants.ClawPosition.OPEN;
                left = RobotConstants.ClawPosition.OPEN;
            } else if (secondary.rightBumperOnce()) {
                left = RobotConstants.ClawPosition.GRAB;
            }

            if(secondary.AOnce()) {
                left = RobotConstants.ClawPosition.MIDDLE;
                right = RobotConstants.ClawPosition.MIDDLE;
            }

            if (-secondary.right_stick_y > DEADZONE) {
                armPosition = RobotConstants.ArmPosition.GRAB;
                wristPosition = RobotConstants.WristPosition.VERTICAL;
            } else if (-secondary.right_stick_y < -DEADZONE)
                armPosition = RobotConstants.ArmPosition.BACK_BOARD;
            else if (secondary.rightStickButtonOnce())
                armPosition = RobotConstants.ArmPosition.TELEOP_POS;
            else if (secondary.XOnce()) {
                armPosition = RobotConstants.ArmPosition.DOWN;
                wristPosition = RobotConstants.WristPosition.VERTICAL;
            }

            if(secondary.dpadUpOnce()) {
                wristPosition = RobotConstants.WristPosition.VERTICAL;
            } else if(secondary.dpadRightOnce()) {
                wristPosition = RobotConstants.WristPosition.RIGHT_HORIZONTAL;
            } else if(secondary.dpadLeftOnce()) {
                wristPosition = RobotConstants.WristPosition.LEFT_HORIZONTAL;
            } else if(secondary.touchpad_finger_1()) {
                wristPosition = RobotConstants.WristPosition.MANUAL;
            }

            if(secondary.dpadDownOnce()) {
                stateMachine.changeState(Executive.StateMachine.StateType.ARM, new AutoTransfer());
            }

            double liftPower = secondary.Y() ? liftSpeed : (secondary.B() ? -liftSpeed : 0.0);
            RobotConfiguration.LIFT.getAsMotor().setPower(liftPower);

            RobotConfiguration.ARM_LEFT.getAsServo().setPosition(armPosition.getLeftPos());
            RobotConfiguration.ARM_RIGHT.getAsServo().setPosition(armPosition.getRightPos());

            RobotConfiguration.WRIST.getAsServo().setPosition(wristPosition.equals(RobotConstants.WristPosition.MANUAL) ?
                    (secondary.touchpad_finger_1_x + 1.0) / 2.0 * 1.08 : wristPosition.getPosition());

            RobotConfiguration.CLAW_LEFT.getAsServo().setPosition(left.getLeftPos());
            RobotConfiguration.CLAW_RIGHT.getAsServo().setPosition(right.getRightPos());
        }
    }

    class Slide_Position extends Executive.StateBase<Manual> {
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
        public void init(Executive.StateMachine<Manual> stateMachine) {
            super.init(stateMachine);
            profileStart = stateTimer.seconds();
            MotionState start = new MotionState(slides.getCurrentPosition(), 0, 0, 0);
            MotionState goal = new MotionState(setpoint, 0, 0, 0);
            activeProfile = MotionProfileGenerator.generateSimpleMotionProfile(start, goal, 1400, 900);
        }

        @Override
        public void update() {
            super.update();

            if(Math.abs(secondary.left_stick_y) > 0.2) {
                stateMachine.changeState(Executive.StateMachine.StateType.SLIDES, new Slide_Manual());
                profileStart = 0;
                activeProfile = null;
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

    class Slide_Speed extends Executive.StateBase<Manual> {
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

    class Slide_Manual extends Executive.StateBase<Manual> {

        private final Motor left = RobotConfiguration.SLIDE_LEFT.getAsMotor(), right = RobotConfiguration.SLIDE_RIGHT.getAsMotor();
        @Override
        public void update() {
            super.update();
            if(Math.abs(secondary.left_stick_y) < 0.2) {
                stateMachine.changeState(Executive.StateMachine.StateType.SLIDES, new Slide_Position(RobotConfiguration.LIFT_ENCODER.getAsEncoder().getCurrentPosition()));
                return;
            }
            double power = -secondary.left_stick_y < 0 ? -secondary.left_stick_y * slideDownSpeed : -secondary.left_stick_y * slideSpeed;
            left.setPower(power);
            right.setPower(power);
        }
    }

    class DroneLaunch extends Executive.StateBase<Manual> {
        private final Servo ramp = RobotConfiguration.RAMP.getAsServo();
        @Override
        public void init(Executive.StateMachine<Manual> stateMachine) {
            super.init(stateMachine);
            ramp.setPosition(IntakePosition.INTAKE.getPosition());
        }

        @Override
        public void update() {
            super.update();
            if(stateTimer.seconds() > 0.75) {
                RobotConfiguration.DRONE.getAsServo().setPosition(0.5);
            } else if(stateTimer.seconds() > 1.0)
                stateMachine.removeStateByType(Executive.StateMachine.StateType.INTAKE);
        }
    }

    class AutoTransfer extends Executive.StateBase<Manual> {
        private boolean bool = false, bool2 = false, bool3 = false, bool4 = false;
        @Override
        public void init(Executive.StateMachine<Manual> stateMachine) {
            super.init(stateMachine);
            stateMachine.changeState(Executive.StateMachine.StateType.SLIDES, new Slide_Position(50));
            left = RobotConstants.ClawPosition.OPEN;
            right = RobotConstants.ClawPosition.OPEN;
        }

        @Override
        public void update() {
            super.update();
            if(!wristPosition.equals(RobotConstants.WristPosition.VERTICAL) && !(Math.abs(secondary.left_stick_y) < 0.2) && !(Math.abs(secondary.right_stick_y) < 0.2))
                stateMachine.removeStateByType(Executive.StateMachine.StateType.ARM);

            if(stateTimer.seconds() > 0.5 && !bool && stateMachine.getStateReferenceByType(Executive.StateMachine.StateType.SLIDES).isDone) {
                armPosition = RobotConstants.ArmPosition.DOWN;
                bool = true;
            } else if(stateTimer.seconds() > 0.08 && !bool2 && bool) {
                stateTimer.reset();
                bool2 = true;
            } else if(stateTimer.seconds() > 0.4 && !bool3 && bool2) {
                left = RobotConstants.ClawPosition.GRAB;
                right = RobotConstants.ClawPosition.GRAB;
                bool3 = true;
            } else if(stateTimer.seconds() > 0.58 && !bool4 && bool3) {
                stateMachine.changeState(Executive.StateMachine.StateType.SLIDES, new Slide_Position(80));
                armPosition = RobotConstants.ArmPosition.TELEOP_POS;
                stateMachine.removeStateByType(Executive.StateMachine.StateType.ARM);
                bool4 = true;
            }
        }
    }
}
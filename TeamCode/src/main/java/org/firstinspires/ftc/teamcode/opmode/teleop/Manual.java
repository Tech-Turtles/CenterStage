package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.core.RobotConstants.DEADZONE;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.IntakePosition;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.OUTTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.WRIST_CENTER;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.utility.autonomous.Executive;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Translation2d;

@Config
@TeleOp(name="Manual", group="A")
public class Manual extends RobotHardware {
    public static double precisionMode = 1.0;
    private final double precisionPercentage = 0.35;
    public static boolean fieldRelative = false;
    public static boolean headingCorrection = true;
    private final Executive.StateMachine<Manual> stateMachine;

    private RobotConstants.ClawPosition left = RobotConstants.ClawPosition.OPEN, right = RobotConstants.ClawPosition.OPEN;
    private RobotConstants.ArmPosition armPosition = RobotConstants.ArmPosition.START;

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
    }

    @Override
    public void init_loop() {
        super.init_loop();
        stateMachine.update();
        RobotConfiguration.RAMP.getAsServo().setPosition(IntakePosition.START.getPosition());
        RobotConfiguration.ARM_LEFT.getAsServo().setPosition(armPosition.getLeftPos());
        RobotConfiguration.ARM_RIGHT.getAsServo().setPosition(armPosition.getRightPos());
        RobotConfiguration.WRIST.getAsServo().setPosition(WRIST_CENTER);
    }

    @Override
    public void start() {
        super.start();
        stateMachine.changeState(Executive.StateMachine.StateType.DRIVE, new Drive_Manual());

        if(fieldRelative)
            primary.setLedColor(0.0, 0.0, 1.0, -1);
        else
            primary.setLedColor(1.0, 1.0, 0.0, -1);
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
                if(fieldRelative)
                    primary.setLedColor(0.0, 0.0, 1.0, -1);
                else
                    primary.setLedColor(1.0, 1.0, 0.0, -1);
            }

            if (primary.XOnce())
                headingCorrection = !headingCorrection;

            if(primary.rightStickButtonOnce()) {
                SwerveDrive.lastHeadingRadians = (3.0 * Math.PI) / 2.0;
                SwerveDrive.updatedHeading = true;
            }

            double xV = -primary.left_stick_y * swerveControllerConfiguration.maxSpeed * precisionMode;
            double yV = -primary.left_stick_x * swerveControllerConfiguration.maxSpeed * precisionMode;
            double thetaV = -primary.right_stick_x * swerveControllerConfiguration.maxAngularVelocity * precisionMode;

            swerveDrive.drive(new Translation2d(xV, yV), thetaV, fieldRelative, true, headingCorrection);
            swerveDrive.updateOdometry();

            if(primary.right_trigger > DEADZONE) {
                RobotConfiguration.INTAKE.getAsMotor().setPower(INTAKE_SPEED * primary.right_trigger);
                RobotConfiguration.RAMP.getAsServo().setPosition(IntakePosition.INTAKE.getPosition());
            } else if(primary.left_trigger > DEADZONE) {
                RobotConfiguration.INTAKE.getAsMotor().setPower(OUTTAKE_SPEED * primary.left_trigger);
                RobotConfiguration.RAMP.getAsServo().setPosition(IntakePosition.INTAKE.getPosition());
            } else if(primary.dpadUp()) {
                RobotConfiguration.INTAKE.getAsMotor().setPower(OUTTAKE_SPEED);
                RobotConfiguration.RAMP.getAsServo().setPosition(IntakePosition.DRIVE.getPosition());
            } else {
                RobotConfiguration.INTAKE.getAsMotor().setPower(0.0);
                RobotConfiguration.RAMP.getAsServo().setPosition(IntakePosition.DRIVE.getPosition());
            }

            if(secondary.rightTriggerOnce()) {
                right = right.equals(RobotConstants.ClawPosition.OPEN) ? RobotConstants.ClawPosition.CLOSE : RobotConstants.ClawPosition.OPEN;
            } else if(secondary.rightBumperOnce()) {
                right = RobotConstants.ClawPosition.CLOSE;
            }

            if(secondary.leftTriggerOnce()) {
                left = left.equals(RobotConstants.ClawPosition.OPEN) ? RobotConstants.ClawPosition.CLOSE : RobotConstants.ClawPosition.OPEN;
            } else if(secondary.leftBumperOnce()) {
                left = RobotConstants.ClawPosition.CLOSE;
            }

            if(-secondary.left_stick_y > DEADZONE) {
                armPosition = RobotConstants.ArmPosition.BACK_BOARD;
            } else if(-secondary.left_stick_y < -DEADZONE) {
                armPosition = RobotConstants.ArmPosition.GRAB;
            } else if(-secondary.right_stick_y > DEADZONE) {
                armPosition = RobotConstants.ArmPosition.HOLD;
            }

            RobotConfiguration.ARM_LEFT.getAsServo().setPosition(armPosition.getLeftPos());
            RobotConfiguration.ARM_RIGHT.getAsServo().setPosition(armPosition.getRightPos());

            RobotConfiguration.CLAW_LEFT.getAsServo().setPosition(left.getLeftPos());
            RobotConfiguration.CLAW_RIGHT.getAsServo().setPosition(right.getRightPos());
            RobotConfiguration.WRIST.getAsServo().setPosition(WRIST_CENTER);
        }
    }
}
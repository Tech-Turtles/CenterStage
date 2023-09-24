package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.core.RobotConstants.DEADZONE;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.IntakePosition;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.OUTTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.SWERVE_MAX_SPEED;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.SWERVE_PRECISION_SPEED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.utility.autonomous.Executive;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Translation2d;


@Config
@TeleOp(name="Manual", group="A")
public class Manual extends RobotHardware {
    private boolean precisionMode = false;
    public static boolean fieldRelative = false;
    private final Executive.StateMachine<Manual> stateMachine;

    public Manual() {
        stateMachine = new Executive.StateMachine<>(this);
        stateMachine.update();
    }

    @Override
    public void init() {
        super.init();
        stateMachine.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        stateMachine.update();
    }

    @Override
    public void start() {
        super.start();
        stateMachine.changeState(Executive.StateMachine.StateType.DRIVE, new Drive_Manual());
    }

    @Override
    public void loop() {
        super.loop();
        stateMachine.update();

        telemetry.addData("Field Relative", fieldRelative);
        telemetry.addData("Gyro", swerveDrive.getGyroRotation3d().getX());
    }
    
    class Drive_Manual extends Executive.StateBase<Manual> {
        @Override
        public void update() {
            super.update();

            if (primary.AOnce()) {
                swerveDrive.setMaximumSpeed(precisionMode ? SWERVE_MAX_SPEED : SWERVE_PRECISION_SPEED);
                precisionMode = !precisionMode;
            }

            if (primary.YOnce())
                swerveDrive.zeroGyro();

            if (primary.BOnce())
                fieldRelative = !fieldRelative;

            double xVelocity   = -primary.left_stick_y * swerveControllerConfiguration.maxSpeed;
            double yVelocity   = -primary.left_stick_x * swerveControllerConfiguration.maxSpeed;
            double angVelocity = primary.right_stick_x * swerveControllerConfiguration.maxAngularVelocity;

            swerveDrive.drive(new Translation2d(xVelocity, yVelocity), angVelocity, fieldRelative, true);
            swerveDrive.updateOdometry();

//            if(primary.right_trigger > DEADZONE) {
//                RobotConfiguration.INTAKE.getAsMotor().setPower(INTAKE_SPEED);
//                RobotConfiguration.RAMP.getAsServo().setPosition(IntakePosition.INTAKE.getPosition());
//            } else if(primary.left_trigger > DEADZONE) {
//                RobotConfiguration.INTAKE.getAsMotor().setPower(OUTTAKE_SPEED);
//                RobotConfiguration.RAMP.getAsServo().setPosition(IntakePosition.INTAKE.getPosition());
//            } else {
//                RobotConfiguration.INTAKE.getAsMotor().setPower(0.0);
//                RobotConfiguration.RAMP.getAsServo().setPosition(IntakePosition.DRIVE.getPosition());
//            }
        }
    }
}
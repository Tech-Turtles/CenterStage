package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.utility.autonomous.Executive;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Translation2d;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.SWERVE_MAX_SPEED;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.SWERVE_PRECISION_SPEED;


@Config
@TeleOp(name="Manual", group="A")
public class Manual extends RobotHardware {

    public static double linearSpeed = 1.0, lateralSpeed = 1.0, rotationSpeed = 1.0;
    private boolean precisionMode = false;

    public static Pose2d startingPosition = new Pose2d();

    private final Executive.StateMachine<Manual> stateMachine;

    public static boolean fieldRelative = false;

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

            telemetry.addData("Gyro", swerveDrive.getGyroRotation3d().getX());
            telemetry.addData("Field Relative", fieldRelative);

            double xVelocity   = -primary.left_stick_y * swerveControllerConfiguration.maxSpeed;
            double yVelocity   = -primary.left_stick_x * swerveControllerConfiguration.maxSpeed;
            double angVelocity = primary.right_stick_x * swerveControllerConfiguration.maxAngularVelocity;

            swerveDrive.drive(new Translation2d(xVelocity, yVelocity), angVelocity, fieldRelative, true);
        }
    }
}
package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.utility.autonomous.Executive;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Translation2d;


@Config
@TeleOp(name="Manual", group="A")
public class Manual extends RobotHardware {

    public static double linearSpeed = 1.0, lateralSpeed = 1.0, rotationSpeed = 1.0;
    private double precisionMode = 1.0;

    double theta = Math.toRadians(0.0);

    public static Pose2d startingPosition = new Pose2d();

    private final Executive.StateMachine<Manual> stateMachine;

    private boolean fieldCentric = true;
    private final double precisionPercentage = 0.35;

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

            if (primary.AOnce())
                precisionMode = precisionMode == 1.0 ? precisionPercentage : 1.0;

            if (primary.YOnce())
                swerveDrive.setGyro(new Rotation3d());

            if (primary.BOnce())
                fieldCentric = !fieldCentric;

            double xVelocity   = -primary.left_stick_y * swerveControllerConfiguration.maxSpeed * precisionMode;
            double yVelocity   = -primary.left_stick_x * swerveControllerConfiguration.maxSpeed * precisionMode;
            double angVelocity = primary.right_stick_x * swerveControllerConfiguration.maxAngularVelocity * precisionMode;

            swerveDrive.drive(new Translation2d(xVelocity, yVelocity), angVelocity, fieldCentric, true);
        }
    }
}
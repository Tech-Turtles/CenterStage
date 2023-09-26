package org.firstinspires.ftc.teamcode.swerve;

import org.firstinspires.ftc.teamcode.utility.math.ElapsedTimer;
import org.firstinspires.ftc.teamcode.utility.math.controller.HolonomicDriveController;
import org.firstinspires.ftc.teamcode.utility.math.controller.PIDController;
import org.firstinspires.ftc.teamcode.utility.math.controller.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.utility.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.utility.math.trajectory.Trajectory;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link Trajectory} with a swerve drive.
 *
 * <p>This command outputs the raw desired Swerve Module States ({@link SwerveModuleState2}) in an
 * array. The desired wheel and module rotation velocities should be taken from those and used in
 * velocity PIDs.
 *
 * <p>The robot angle controller does not follow the angle given by the trajectory but rather goes
 * to the angle given in the final state of the trajectory.
 */
public class SwerveDriveController {
    private final ElapsedTimer m_timer = new ElapsedTimer();
    private final Trajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final SwerveKinematics2 m_kinematics;
    private final HolonomicDriveController m_controller;
    private final Consumer<SwerveModuleState2[]> m_outputModuleStates;
    private final Supplier<Rotation2d> m_desiredRotation;

    /**
     * Constructs a new SwerveController that when executed will follow the provided
     * trajectory. This command will not return output voltages but rather raw module states from the
     * position controllers which need to be put into a velocity PID.
     *
     * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path.
     * This is left to the user to do since it is not appropriate for paths with non-stationary
     * endstates.
     *
     * @param trajectory The trajectory to follow.
     * @param pose A function that supplies the robot pose - use one of the odometry classes to
     *     provide this.
     * @param kinematics The kinematics for the robot drivetrain.
     * @param xController The Trajectory Tracker PID controller for the robot's x position.
     * @param yController The Trajectory Tracker PID controller for the robot's y position.
     * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
     * @param desiredRotation The angle that the drivetrain should be facing. This is sampled at each
     *     time step.
     * @param outputModuleStates The raw output module states from the position controllers.
     */
    public SwerveDriveController(
            Trajectory trajectory,
            Supplier<Pose2d> pose,
            SwerveKinematics2 kinematics,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            Supplier<Rotation2d> desiredRotation,
            Consumer<SwerveModuleState2[]> outputModuleStates) {
        this(
                trajectory,
                pose,
                kinematics,
                new HolonomicDriveController(
                        xController,
                        yController,
                        thetaController),
                desiredRotation,
                outputModuleStates);
    }

    /**
     * Constructs a new SwerveController that when executed will follow the provided
     * trajectory. This command will not return output voltages but rather raw module states from the
     * position controllers which need to be put into a velocity PID.
     *
     * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path.
     * This is left to the user since it is not appropriate for paths with nonstationary endstates.
     *
     * <p>Note 2: The final rotation of the robot will be set to the rotation of the final pose in the
     * trajectory. The robot will not follow the rotations from the poses at each timestep. If
     * alternate rotation behavior is desired, the other constructor with a supplier for rotation
     * should be used.
     *
     * @param trajectory The trajectory to follow.
     * @param pose A function that supplies the robot pose - use one of the odometry classes to
     *     provide this.
     * @param kinematics The kinematics for the robot drivetrain.
     * @param xController The Trajectory Tracker PID controller for the robot's x position.
     * @param yController The Trajectory Tracker PID controller for the robot's y position.
     * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
     * @param outputModuleStates The raw output module states from the position controllers.
     */
    public SwerveDriveController(
            Trajectory trajectory,
            Supplier<Pose2d> pose,
            SwerveKinematics2 kinematics,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            Consumer<SwerveModuleState2[]> outputModuleStates) {
        this(
                trajectory,
                pose,
                kinematics,
                xController,
                yController,
                thetaController,
                () ->
                        trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
                outputModuleStates);
    }

    /**
     * Constructs a new SwerveController that when executed will follow the provided
     * trajectory. This command will not return output voltages but rather raw module states from the
     * position controllers which need to be put into a velocity PID.
     *
     * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
     * this is left to the user, since it is not appropriate for paths with non-stationary endstates.
     *
     * <p>Note 2: The final rotation of the robot will be set to the rotation of the final pose in the
     * trajectory. The robot will not follow the rotations from the poses at each timestamp. If
     * alternate rotation behavior is desired, the other constructor with a supplier for rotation
     * should be used.
     *
     * @param trajectory The trajectory to follow.
     * @param pose A function that supplies the robot pose - use one of the odometry classes to
     *     provide this.
     * @param kinematics The kinematics for the robot drivetrain.
     * @param controller The HolonomicDriveController for the drivetrain.
     * @param outputModuleStates The raw output module states from the position controllers.
     */
    public SwerveDriveController(
            Trajectory trajectory,
            Supplier<Pose2d> pose,
            SwerveKinematics2 kinematics,
            HolonomicDriveController controller,
            Consumer<SwerveModuleState2[]> outputModuleStates) {
        this(
                trajectory,
                pose,
                kinematics,
                controller,
                () ->
                        trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
                outputModuleStates);
    }

    /**
     * Constructs a new SwerveController that when executed will follow the provided
     * trajectory. This command will not return output voltages but rather raw module states from the
     * position controllers which need to be put into a velocity PID.
     *
     * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
     * this is left to the user, since it is not appropriate for paths with non-stationary endstates.
     *
     * @param trajectory The trajectory to follow.
     * @param pose A function that supplies the robot pose - use one of the odometry classes to
     *     provide this.
     * @param kinematics The kinematics for the robot drivetrain.
     * @param controller The HolonomicDriveController for the drivetrain.
     * @param desiredRotation The angle that the drivetrain should be facing. This is sampled at each
     *     time step.
     * @param outputModuleStates The raw output module states from the position controllers.
     */
    public SwerveDriveController(
            Trajectory trajectory,
            Supplier<Pose2d> pose,
            SwerveKinematics2 kinematics,
            HolonomicDriveController controller,
            Supplier<Rotation2d> desiredRotation,
            Consumer<SwerveModuleState2[]> outputModuleStates) {
        m_trajectory = trajectory;
        m_pose = pose;
        m_kinematics = kinematics;
        m_controller = controller;

        m_desiredRotation = desiredRotation;

        m_outputModuleStates = outputModuleStates;
    }

    public void initialize() {
        m_timer.reset();
    }

    public void update() {
        double curTime = m_timer.seconds();
        Trajectory.State desiredState = m_trajectory.sample(curTime);

        ChassisSpeeds targetChassisSpeeds =
                m_controller.calculate(m_pose.get(), desiredState, m_desiredRotation.get());
        SwerveModuleState2[] targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

        m_outputModuleStates.accept(targetModuleStates);
    }

    public boolean isFinished() {
        return m_timer.seconds() >= m_trajectory.getTotalTimeSeconds();
    }
}
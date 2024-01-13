package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Encoder;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Servo;

@TeleOp
@Config
public class SlidePIDTest extends RobotHardware {

    PIDCoefficients coeffs;
    PIDFController controller;
    NanoClock clock;
    MotionProfile activeProfile;
    boolean movingForwards;
    double profileStart;

    public static double kP = 0.03, kI = 0.0, kD = 0.0, kV = 0.0, kA = 0.0, kStatic = 0.17, setpoint = 400.0, targetVelo = 1000, targetAccel = 700, kG = 0.04;

    private double prevKP = 0, prevKI = 0, prevKD = 0, prevKV = 0.0, prevKA = 0.0, prevKStatic = 0.0;

    private final Motor left = RobotConfiguration.SLIDE_LEFT.getAsMotor(),
            right = RobotConfiguration.SLIDE_RIGHT.getAsMotor();
    private final Servo armLeft = RobotConfiguration.ARM_LEFT.getAsServo(), armRight = RobotConfiguration.ARM_RIGHT.getAsServo();
    private final Encoder lift = RobotConfiguration.LIFT_ENCODER.getAsEncoder();

    @Override
    public void init() {
        super.init();
        coeffs = new PIDCoefficients(kP, kI, kD);
        controller = new PIDFController(coeffs, kV, kA, kStatic);
//        controller = new PIDFController(coeffs);
        controller.setTargetVelocity(targetVelo);
        controller.setTargetAcceleration(targetAccel);
        armLeft.setPosition(RobotConstants.ArmPosition.HOLD.getLeftPos());
        armRight.setPosition(RobotConstants.ArmPosition.HOLD.getRightPos());
    }

    @Override
    public void start() {
        super.start();
        clock = NanoClock.system();
        movingForwards = true;
        MotionState start = new MotionState(0, 0, 0, 0);
        MotionState goal = new MotionState(setpoint, 0, 0, 0);
        activeProfile = MotionProfileGenerator.generateSimpleMotionProfile(start, goal, targetVelo, targetAccel);
        profileStart = clock.seconds();
        armLeft.setPosition(RobotConstants.ArmPosition.HOLD.getLeftPos());
        armRight.setPosition(RobotConstants.ArmPosition.HOLD.getRightPos());
    }

    @Override
    public void loop() {
        super.loop();


        // calculate and set the motor power
        double profileTime = clock.seconds() - profileStart;

        if (profileTime > activeProfile.duration()) {
            // generate a new profile
            movingForwards = !movingForwards;
            activeProfile = generateProfile(movingForwards);
            profileStart = clock.seconds();
        }

        MotionState motionState = activeProfile.get(profileTime);

        controller.setTargetPosition(motionState.getX());
        controller.setTargetVelocity(motionState.getV());
        controller.setTargetAcceleration(motionState.getA());

        double power = controller.update(lift.getCurrentPosition(), lift.getRawVelocity());
        power = power > 0.01 ? power + kG : power;

        left.setPower(power);
        right.setPower(power);

        packet.put("Slide Velo", lift.getCorrectedVelocity());
        packet.put("Slide Pos", lift.getCurrentPosition());
        packet.put("Target Velo", motionState.getV());

        if (prevKP != kP || prevKI != kI || prevKD != kD || prevKV != kV || prevKA != kA || prevKStatic != kStatic) {
            coeffs = new PIDCoefficients(kP, kI, kD);
            controller = new PIDFController(coeffs, kV, kA, kStatic);
//            controller = new PIDFController(coeffs);
            prevKP = kP;
            prevKI = kI;
            prevKD = kD;
            prevKV = kV;
            prevKA = kA;
            prevKStatic = kStatic;
        }
    }

    private static MotionProfile generateProfile(boolean movingForward) {
        MotionState start = new MotionState(movingForward ? 150 : setpoint, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? setpoint : 150, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, targetVelo, targetAccel);
    }
}
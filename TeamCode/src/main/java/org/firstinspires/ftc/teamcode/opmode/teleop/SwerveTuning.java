package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.utility.math.ElapsedTimer;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Translation2d;

@Config
@TeleOp(name = "Swerve Tuning", group = "E")
public class SwerveTuning extends RobotHardware {
    public static double
        fLOffset = 158.5, fROffset = 121.0, bLOffset = 311.2, bROffset = 78.0;

    public static TuneState state = TuneState.OFFSET;
    public static WheelPosition angleWheel = WheelPosition.FRONT_LEFT;

    public static double aP = 0.08, aI = 0.0, aD = 0.0, aFF = 0.0, angleSetpointStart = 0.0, angleSetpointEnd = 30.0;
    private boolean reverse = false;
    private ElapsedTimer changeDirection;
    public static double changeDirectionTime = 1.0;

    enum WheelPosition{
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

    enum TuneState {
        DRIVE,
        OFFSET,
        ANGLE_PID,
        DRIVE_PID
    }
    private AbsoluteEncoder frontLeft, frontRight, backLeft, backRight;

    @Override
    public void init() {
        super.init();
        frontLeft = RobotConfiguration.ABSOLUTE_FRONT_LEFT.getAsAbsoluteEncoder();
        frontRight = RobotConfiguration.ABSOLUTE_FRONT_RIGHT.getAsAbsoluteEncoder();
        backLeft = RobotConfiguration.ABSOLUTE_BACK_LEFT.getAsAbsoluteEncoder();
        backRight = RobotConfiguration.ABSOLUTE_BACK_RIGHT.getAsAbsoluteEncoder();

        swerveDrive.setMaximumSpeed(1.3);
        changeDirection = new ElapsedTimer();
        changeDirection.reset();
    }

    @Override
    public void loop() {
        super.loop();
        switch(state) {
            case OFFSET:
                frontLeft.zero(fLOffset);
                frontRight.zero(fROffset);
                backLeft.zero(bLOffset);
                backRight.zero(bROffset);
            case DRIVE:
                double xVelocity = -primary.left_stick_y * swerveControllerConfiguration.maxSpeed;
                double yVelocity = -primary.left_stick_x * swerveControllerConfiguration.maxSpeed;
                double angVelocity = primary.right_stick_x * swerveControllerConfiguration.maxAngularVelocity;

                swerveDrive.drive(new Translation2d(xVelocity, yVelocity), angVelocity, false, true);
                break;
            case ANGLE_PID:
                ContinuousServo servo;
                switch (angleWheel) {
                    case FRONT_LEFT:
                        servo = RobotConfiguration.ANGLE_FRONT_LEFT.getAsContinuousServo();
                        break;
                    case FRONT_RIGHT:
                        servo = RobotConfiguration.ANGLE_FRONT_RIGHT.getAsContinuousServo();
                        break;
                    case BACK_LEFT:
                        servo = RobotConfiguration.ANGLE_BACK_LEFT.getAsContinuousServo();
                        break;
                    default:
                        servo = RobotConfiguration.ANGLE_BACK_RIGHT.getAsContinuousServo();
                }
                servo.configurePIDF(aP, aI, aD);

                if(reverse) {
                    if(changeDirection.seconds() > changeDirectionTime) {
                        changeDirection.reset();
                        reverse = false;
                    }
                    servo.setReference(angleSetpointStart, aFF);
                } else {
                    if(changeDirection.seconds() > changeDirectionTime) {
                        changeDirection.reset();
                        reverse = true;
                    }
                    servo.setReference(angleSetpointEnd, aFF);
                }
                break;
        }
        telemetry.addData("Front Left Position", frontLeft.getCurrentPosition());
        telemetry.addData("Front Right Position", frontRight.getCurrentPosition());
        telemetry.addData("Back Left Position", backLeft.getCurrentPosition());
        telemetry.addData("Back Right Position", backRight.getCurrentPosition());
    }
}
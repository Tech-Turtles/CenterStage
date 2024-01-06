package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.hardware.Encoder;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareDevice;
import org.firstinspires.ftc.teamcode.utility.math.ElapsedTimer;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Translation2d;

@Config
@TeleOp(name = "Swerve Tuning", group = "E")
public class SwerveTuning extends RobotHardware {
    public static double
        fLOffset = 0.0, fROffset = 311.0-2.5, bLOffset = 138.7-18.9-0.6, bROffset = 158.0-2.7;

    public static TuneState state = TuneState.DRIVE;
    public static WheelPosition angleWheel = WheelPosition.FRONT_LEFT;

    public static double aP = 0.08, aI = 0.0, aD = 0.0, aFF = 0.0, angleSetpointStart = 0.0, angleSetpointEnd = 30.0, kStatic = 0.0;
    private double prevAP = aP, prevAI = aI, prevAD = aD, prevAFF = aFF;
    private boolean reverse = false;
    private ElapsedTimer changeDirection;
    public static double changeDirectionTime = 1.0;
    private int index = 0;
    public static boolean precision = false;
    private double precisionMode = 0.35;

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
        DRIVE_PID,
        kStatic
    }
    private AbsoluteEncoder frontLeft, frontRight, backLeft, backRight;
    private ContinuousServo fL, fR, bL, bR;

    @Override
    public void init() {
        super.init();
        frontLeft = RobotConfiguration.ABSOLUTE_FRONT_LEFT.getAsAbsoluteEncoder();
        frontRight = RobotConfiguration.ABSOLUTE_FRONT_RIGHT.getAsAbsoluteEncoder();
        backLeft = RobotConfiguration.ABSOLUTE_BACK_LEFT.getAsAbsoluteEncoder();
        backRight = RobotConfiguration.ABSOLUTE_BACK_RIGHT.getAsAbsoluteEncoder();

        fL = RobotConfiguration.ANGLE_FRONT_LEFT.getAsContinuousServo();
        fR = RobotConfiguration.ANGLE_FRONT_RIGHT.getAsContinuousServo();
        bL = RobotConfiguration.ANGLE_BACK_LEFT.getAsContinuousServo();
        bR = RobotConfiguration.ANGLE_BACK_RIGHT.getAsContinuousServo();

//        swerveDrive.setMaximumSpeed(1.3);
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

                for(RobotConfiguration configuration : RobotConfiguration.values()) {
                    HardwareDevice device = configuration.getAsHardwareDevice();
                    if(device instanceof AbsoluteEncoder)
                        telemetry.addData(configuration.name() + " Abs", ((AbsoluteEncoder) device).getCurrentPosition());

                }
            case DRIVE:
                double xVelocity;
                double yVelocity;
                double angVelocity;
                double prec = precision ? precisionMode : 1.0;
                if (primary.dpadUp()) {
                    xVelocity = swerveControllerConfiguration.maxSpeed * prec;
                    yVelocity = 0.0;
                    angVelocity = 0.0;
                } else if (primary.dpadRight()) {
                    xVelocity = 0.0;
                    yVelocity = swerveControllerConfiguration.maxSpeed * prec;
                    angVelocity = 0.0;
                } else {
                    xVelocity = -primary.left_stick_y * swerveControllerConfiguration.maxSpeed * prec;
                    yVelocity = -primary.left_stick_x * swerveControllerConfiguration.maxSpeed * prec;
                    angVelocity = primary.right_stick_x * swerveControllerConfiguration.maxAngularVelocity * prec;
                }

                swerveDrive.drive(new Translation2d(xVelocity, yVelocity), angVelocity, false, true);
                break;
            case ANGLE_PID:
                if(prevAP != aP || prevAI != aI || prevAD != aD || prevAFF != aFF) {
                    fL.configurePIDF(aP, aI, aD, aFF);
                    fR.configurePIDF(aP, aI, aD, aFF);
                    bL.configurePIDF(aP, aI, aD, aFF);
                    bR.configurePIDF(aP, aI, aD, aFF);
                    prevAP = aP;
                    prevAI = aI;
                    prevAD = aD;
                    prevAFF = aFF;
                    index++;
                }


                double xVelocity$ = -primary.left_stick_y * swerveControllerConfiguration.maxSpeed;
                double yVelocity$ = -primary.left_stick_x * swerveControllerConfiguration.maxSpeed;
                double angVelocity$ = primary.right_stick_x * swerveControllerConfiguration.maxAngularVelocity;

                swerveDrive.drive(new Translation2d(xVelocity$, yVelocity$), angVelocity$, false, true);
                telemetry.addData("Index", index);
                break;
            case kStatic:
                ContinuousServo s;
                switch (angleWheel) {
                    case FRONT_LEFT:
                        s = RobotConfiguration.ANGLE_FRONT_LEFT.getAsContinuousServo();
                        break;
                    case FRONT_RIGHT:
                        s = RobotConfiguration.ANGLE_FRONT_RIGHT.getAsContinuousServo();
                        break;
                    case BACK_LEFT:
                        s = RobotConfiguration.ANGLE_BACK_LEFT.getAsContinuousServo();
                        break;
                    default:
                        s = RobotConfiguration.ANGLE_BACK_RIGHT.getAsContinuousServo();
                }
                s.setPower(kStatic);
                break;
        }
        telemetry.addData("Front Left Position", frontLeft.getCurrentPosition());
        telemetry.addData("Front Right Position", frontRight.getCurrentPosition());
        telemetry.addData("Back Left Position", backLeft.getCurrentPosition());
        telemetry.addData("Back Right Position", backRight.getCurrentPosition());
    }
}
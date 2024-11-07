package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceId.Neo;
import frc.robot.lib.IDashboardProvider;
import frc.robot.DeviceId.Encoder;
import frc.robot.Constants.MotorReverse;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.DriveEncoderReverse;
import frc.robot.Constants.EncoderOffset;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase implements IDashboardProvider {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final AHRS gyro;
    private final SwerveDriveOdometry odometry;

    public SwerveSubsystem() {
        this.registerDashboard();
        this.frontLeft = new SwerveModule(
            Neo.frontLeftDrive,
            Neo.frontLeftTurn,
            Encoder.frontLeft,
            MotorReverse.FRONT_LEFT_DRIVE,
            MotorReverse.FRONT_LEFT_TURN,
            DriveEncoderReverse.FRONT_LEFT,
            EncoderOffset.FRONT_LEFT,
            "frontLeft"
        );
        this.frontRight = new SwerveModule(
            Neo.frontRightDrive,
            Neo.frontRightTurn,
            Encoder.frontRight,
            MotorReverse.FRONT_RIGHT_DRIVE,
            MotorReverse.FRONT_RIGHT_TURN,
            DriveEncoderReverse.FRONT_RIGHT,
            EncoderOffset.FRONT_RIGHT,
            "frontRight"
        );
        this.backLeft = new SwerveModule(
            Neo.backLeftDrive,
            Neo.backLeftTurn,
            Encoder.backLeft,
            MotorReverse.BACK_LEFT_DRIVE,
            MotorReverse.BACK_LEFT_TURN,
            DriveEncoderReverse.BACK_LEFT,
            EncoderOffset.BACK_LEFT,
            "backLeft"
        );
        this.backRight = new SwerveModule(
            Neo.backRightDrive,
            Neo.backRightTurn,
            Encoder.backRight,
            MotorReverse.BACK_RIGHT_DRIVE,
            MotorReverse.BACK_RIGHT_TURN,
            DriveEncoderReverse.BACK_RIGHT,
            EncoderOffset.BACK_RIGHT,
            "backRight"
        );
        this.gyro = new AHRS(SPI.Port.kMXP);
        this.odometry = new SwerveDriveOdometry(
            Constants.swerveDriveKinematics, this.gyro.getRotation2d(), this.getModulePosition()
        );
        this.wait(1000);
        this.gyro.reset();
    }

    @Override
    public void periodic() {
        this.odometry.update(this.gyro.getRotation2d(), getModulePosition());
    }

    public void resetGyro() {
        this.gyro.reset();
    }

    public void driveSwerve(double xSpeed, double ySpeed, double rotation, boolean field) {
        SwerveModuleState[] state = Constants.swerveDriveKinematics.toSwerveModuleStates(field ? 
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, this.gyro.getRotation2d()) :
            new ChassisSpeeds(xSpeed, ySpeed, rotation)
        );
        this.setModuleState(state);
    }

    public void chassisDrive(ChassisSpeeds relativeSpeed) {
        ChassisSpeeds targetSpeed = ChassisSpeeds.discretize(relativeSpeed, 0.02);
        SwerveModuleState state[] = Constants.swerveDriveKinematics.toSwerveModuleStates(targetSpeed);
        this.setModuleState(state);
    }

    public void setModuleState(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
        this.frontLeft.setDesiredState(states[0]);
        this.frontRight.setDesiredState(states[1]);
        this.backLeft.setDesiredState(states[2]);
        this.backRight.setDesiredState(states[3]);
    }

    public SwerveModuleState[] getModuleState() {
        return new SwerveModuleState[] {
            this.frontLeft.getState(),
            this.frontRight.getState(),
            this.backLeft.getState(),
            this.backRight.getState()
        };
    }

    public SwerveModulePosition[] getModulePosition() {
        return new SwerveModulePosition[] {
            this.frontLeft.getPosition(),
            this.frontRight.getPosition(),
            this.backLeft.getPosition(),
            this.backRight.getPosition()
        };
    }

    public void wait(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public Pose2d getPose() {
        return this.odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        this.odometry.resetPosition(this.gyro.getRotation2d(), this.getModulePosition(), pose);
    }


    public ChassisSpeeds getSpeeds() {
        return Constants.swerveDriveKinematics.toChassisSpeeds(this.getModuleState());
    }

    public void stopModules() {
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.backRight.stop();
    }

    @Override
    public void putDashboard() {
    }
}
package frc.robot.subsystems;

//import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final PIDController turningPidController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetDegrees;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffsetDegrees, boolean absoluteEncoderReversed) {

        SupplyCurrentLimitConfiguration currentConfig;

        this.absoluteEncoderOffsetDegrees = absoluteEncoderOffsetDegrees;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderId, "CANivore");

        driveMotor = new TalonFX(driveMotorId, "CANivore");
        turningMotor = new TalonFX(turningMotorId, "CANivore");

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        //TODO: maybe copy the PID + motion magic stuff from the old Falcon500SteerControllerFactoryBuilder.java?

        currentConfig = new SupplyCurrentLimitConfiguration();
        currentConfig.currentLimit = 30;
        currentConfig.enable = true;

        driveMotor.configFactoryDefault();
        driveMotor.setNeutralMode( NeutralMode.Brake );
        driveMotor.configVoltageCompSaturation(12.5);
        driveMotor.enableVoltageCompensation(true);
        driveMotor.configSupplyCurrentLimit( currentConfig );

        turningMotor.configFactoryDefault();
        turningMotor.setNeutralMode( NeutralMode.Brake );
        currentConfig.currentLimit = 15;
        turningMotor.configVoltageCompSaturation(12.5);
        turningMotor.enableVoltageCompensation(true);
        turningMotor.configSupplyCurrentLimit( currentConfig );
    
        absoluteEncoder.configSensorInitializationStrategy( SensorInitializationStrategy.BootToAbsolutePosition );
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        turningMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Signed_PlusMinus180);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        double position;
        position = driveMotor.getSelectedSensorPosition();  // 0..4095 counts per revolution

        return( position );
    }

    public double getTurningPosition() {
        double position;
        position = turningMotor.getSelectedSensorPosition();

        // convert +/- 180 degrees to radians
        position = position * (Math.PI / 180);

        return( position );
    }

    public double getDriveVelocity() {
        double velocity;
        velocity = driveMotor.getSelectedSensorVelocity();

        // convert (0..409.5) counts per 100 ms into radians per second
        velocity = velocity * ((360 * 10) / 4096) * (Math.PI / 180);

        return( velocity );
    }

    public double getTurningVelocity() {
        double velocity;
        velocity = turningMotor.getSelectedSensorPosition();

        // convert degrees/100 milliseconds to radians per second
        velocity = (velocity * 10) * (Math.PI / 180);

        return( velocity );
    }

    public double getAbsoluteEncoderDegrees() {
        double position = absoluteEncoder.getAbsolutePosition();

        position = position - absoluteEncoderOffsetDegrees;

        return position * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        // Clear the drive motor encoder position
        driveMotor.setSelectedSensorPosition(0);

        // Since the turning motor is a relative encoder, set its position equal to the absolute encoder
        turningMotor.setSelectedSensorPosition( getAbsoluteEncoderDegrees() );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set( ControlMode.PercentOutput, 
                        state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond );
        turningMotor.set( ControlMode.PercentOutput, 
                          turningPidController.calculate( getTurningPosition(), state.angle.getRadians() ) );
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set( ControlMode.PercentOutput, 0 );
        turningMotor.set( ControlMode.PercentOutput, 0 );
    }
}

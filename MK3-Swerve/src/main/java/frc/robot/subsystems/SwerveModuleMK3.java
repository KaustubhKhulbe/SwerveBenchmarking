package frc.robot.subsystems;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

public class SwerveModuleMK3 {

  File file;
  FileOutputStream oFile;
  int count = 0;

  // TODO: Tune these PID values for your robot
  private static final double kDriveP = 15.0;
  private static final double kDriveI = 0.01;
  private static final double kDriveD = 0.1;
  private static final double kDriveF = 0.2;

  private static final double kAngleP = 1.0;
  private static final double kAngleI = 0.0;
  private static final double kAngleD = 0.0;

  // CANCoder has 4096 ticks/rotation
  private static double kEncoderTicksPerRotation = 4096;

  private TalonFX driveMotor;
  private TalonFX angleMotor;
  private CANCoder canCoder;
  private Rotation2d offset;

  private long startTime;
  private int numStateChanges = 1000;

  public SwerveModuleMK3(TalonFX driveMotor, TalonFX angleMotor, CANCoder canCoder, Rotation2d offset) {

    try {
      file = new File("/home/lvuser/encoders.txt");
      file.createNewFile();
      oFile = new FileOutputStream(file, false);
    } catch (IOException e) {
      System.out.println("Error setting up file.");
    }

    this.driveMotor = driveMotor;
    this.angleMotor = angleMotor;
    this.canCoder = canCoder;
    this.offset = offset;

    TalonFXConfiguration angleTalonFXConfiguration = new TalonFXConfiguration();

    angleTalonFXConfiguration.slot0.kP = kAngleP;
    angleTalonFXConfiguration.slot0.kI = kAngleI;
    angleTalonFXConfiguration.slot0.kD = kAngleD;

    // Use the CANCoder as the remote sensor for the primary TalonFX PID
    angleTalonFXConfiguration.remoteFilter0.remoteSensorDeviceID = canCoder.getDeviceID();
    angleTalonFXConfiguration.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;

    angleTalonFXConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
    angleMotor.configAllSettings(angleTalonFXConfiguration);
    angleMotor.setNeutralMode(NeutralMode.Coast); // not needed but nice to keep the robot stopped when you want it
                                                  // stopped

    TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();

    driveTalonFXConfiguration.slot0.kP = kDriveP;
    driveTalonFXConfiguration.slot0.kI = kDriveI;
    driveTalonFXConfiguration.slot0.kD = kDriveD;
    driveTalonFXConfiguration.slot0.kF = kDriveF;

    driveMotor.configAllSettings(driveTalonFXConfiguration);
    driveMotor.setNeutralMode(NeutralMode.Coast);

    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.magnetOffsetDegrees = offset.getDegrees();
    canCoder.configAllSettings(canCoderConfiguration);
  }

  /**
   * Gets the relative rotational position of the module
   * 
   * @return The relative rotational position of the angle motor in degrees
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(canCoder.getAbsolutePosition()); // include angle offset
  }

  public double getRawAngle() {
    return canCoder.getAbsolutePosition(); // include angle offset
  }

  // :)
  /**
   * Set the speed + rotation of the swerve module from a SwerveModuleState object
   * 
   * @param desiredState - A SwerveModuleState representing the desired new state
   *                     of the module
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    /**
     * INFO event loop time: 20ms, integrated encoder ticks: 2048, gear ratio: 12.8
     * : 1, velocity: deg/s
     */

    System.out.println(count);
    if (count == 0)
      startTime = System.currentTimeMillis();

    if (count == numStateChanges) {
      try {
        oFile.flush();
        oFile.close();
      } catch (IOException e) {
        e.printStackTrace();
      }
      System.out.println("Success");
    } else if (count > numStateChanges) {
      angleMotor.set(TalonFXControlMode.PercentOutput, 0);
      return;
    } else {
      try {
        String content = angleMotor.getSelectedSensorPosition() + "\t" + canCoder.getAbsolutePosition() + "\t"
            + (System.currentTimeMillis() - startTime) + "\t" + canCoder.getVelocity() + "\t"
            + canCoder.getLastTimestamp() + "\n";
        oFile.write(content.getBytes());
      } catch (IOException e) {
        System.out.println("Error writing to file.");
      }
    }

    Rotation2d currentRotation = getAngle();
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);

    // Find the difference between our current rotational position + our new
    // rotational position
    Rotation2d rotationDelta = state.angle.minus(currentRotation);

    // Find the new absolute position of the module based on the difference in
    // rotation
    double deltaTicks = (rotationDelta.getDegrees() / 360) * kEncoderTicksPerRotation;
    // Convert the CANCoder from it's position reading back to ticks
    double currentTicks = canCoder.getPosition() / canCoder.configGetFeedbackCoefficient();
    double desiredTicks = currentTicks + deltaTicks;

    // below is a line to comment out from step 5
    // angleMotor.set(TalonFXControlMode.Position, desiredTicks);
    angleMotor.set(TalonFXControlMode.PercentOutput, 0.2);

    double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond);

    // below is a line to comment out from step 5
    driveMotor.set(TalonFXControlMode.PercentOutput, 0);
    // pw.write(angleMotor.getSelectedSensorPosition() + ", " +
    // canCoder.getAbsolutePosition());

    count++;
  }

}

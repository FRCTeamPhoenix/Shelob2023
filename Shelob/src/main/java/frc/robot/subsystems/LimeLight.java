package frc.robot.subsystems;


//import frc.robot.commands.*;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {

private boolean m_LimelightHasValidTarget = false;
private double m_LimelightDriveCommand = 0.0;
private double m_LimelightSteerCommand = 0.0;
private double m_targetArea = 0.0;

    public void limeLight() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Update_Limelight_Tracking();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void Update_Limelight_Tracking(){
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.1;                  // How hard to turn toward the target
        final double DRIVE_K = 1;                    // How hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 1;        // Area of the target when the robot reaches the wall
        final double MAX_FORWARD_DRIVE = -0.7;   
        final double MAX_REVERSE_DRIVE = 0.7;        // Simple speed limit so we don't drive too fast

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        //double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        m_targetArea = ta;

        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // Try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // Don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_REVERSE_DRIVE)
        {
          drive_cmd = MAX_REVERSE_DRIVE;
        }
        if (drive_cmd < MAX_FORWARD_DRIVE)
        {
          drive_cmd = MAX_FORWARD_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
  }

  public boolean hasValidTarget() {
      return m_LimelightHasValidTarget;
  }

  public double getLLDriveSpeed() {
    return m_LimelightDriveCommand;
  }

  public double getLLTurnSpeed() {
    return m_LimelightSteerCommand;
  }

  public double getLLTargetArea() {
    return m_targetArea;
  }
}


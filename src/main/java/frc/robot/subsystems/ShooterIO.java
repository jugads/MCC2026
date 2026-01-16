package frc.robot.subsystems;

public interface ShooterIO {
     default void setShooterSpeed (double speed) {}

      public class shooterIOInputs {
        public double ShooterSpeed = 0.0;
    }
    default void updateInputs(shooterIOInputs inputs) {}
    default void refreshData() {}
}



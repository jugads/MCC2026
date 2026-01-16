package frc.robot.subsystems;

public interface HopperIO {
    default void setBeltspeed (double speed) {}
    
    public class HopperIOInputs {
        public double beltspeed = 0.0;
    }

    default void updateInputs(HopperIOInputs inputs) {}
    default void refreshData() {}

}

 
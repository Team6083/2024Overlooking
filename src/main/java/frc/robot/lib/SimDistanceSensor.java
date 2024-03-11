package frc.robot.lib;

public class SimDistanceSensor implements DistanceSensorInterface {

    public SimDistanceSensor(){

    }

    @Override
    public double getTargetDistance() {
        return -1;
    }

    @Override
    public boolean isGetTarget() {
       return true;
    }

    @Override
    public void setAutomaticMode(boolean isEnable) {
        return;
    }
    
}

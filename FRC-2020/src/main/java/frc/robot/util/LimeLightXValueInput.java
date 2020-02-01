package frc.robot.input;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import frc.robot.Robot;

public class LimeLightXValueInput implements PIDSource{
    private PIDSourceType sourceType;

	//Wrapper class to get X Values from the LimeLightSubsystem
	public LimeLightXValueInput() {
        sourceType = PIDSourceType.kDisplacement;
    }
    
    @Override
    public PIDSourceType getPIDSourceType() {
        return sourceType;
    }

    @Override
    public double pidGet() {
        if(sourceType == PIDSourceType.kDisplacement) 
            return Robot.sensorSubystem.getX();
        else 
            return 0;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        sourceType = pidSource;
    }
}

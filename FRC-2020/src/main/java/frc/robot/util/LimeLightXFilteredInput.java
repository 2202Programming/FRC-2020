package frc.robot.input;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.filters.LinearDigitalFilter;
import frc.robot.Robot;

public class LimeLightXFilteredInput implements PIDSource{
    private PIDSourceType sourceType;
    private LinearDigitalFilter lowPassFilter;

	//Wrapper class to get X Values from the LimeLightSubsystem
	public LimeLightXFilteredInput() {
        sourceType = PIDSourceType.kDisplacement;
        lowPassFilter = LinearDigitalFilter.movingAverage(new LimeLightXValueInput(), 5);
    }
    
    @Override
    public PIDSourceType getPIDSourceType() {
        return sourceType;
    }

    @Override
    public double pidGet() {
        if(sourceType == PIDSourceType.kDisplacement) 
            return lowPassFilter.pidGet();
        else 
            return 0;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        sourceType = pidSource;
    }
}

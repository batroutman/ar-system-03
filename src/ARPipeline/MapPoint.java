package ARPipeline;

import org.opencv.core.Mat;

public class MapPoint {

	// aposteriori
	protected Double u = null;
	protected Double v = null;
	
	// apriori
	protected Double x = null;
	protected Double y = null;
	protected Double z = null;
	
	// initial descriptor
	protected Mat descriptor = null;
	
	public MapPoint() {
		
	}
	
	public MapPoint(Double u, Double v) {
		
	}
	
}

package ARPipeline;

import java.util.ArrayList;

import org.opencv.core.Mat;

public class MapPoint {

	protected Point3D point = null;
	protected ArrayList<Observation> observations = new ArrayList<Observation>();
	protected Mat principalDescriptor = null;

	public MapPoint() {

	}

	public MapPoint(Observation observation) {
		this.observations.add(observation);
		this.principalDescriptor = observation.getDescriptor().clone();
	}

	public Point3D getPoint() {
		return point;
	}

	public void setPoint(Point3D point) {
		this.point = point;
	}

	public ArrayList<Observation> getObservations() {
		return observations;
	}

	public void setObservations(ArrayList<Observation> observations) {
		this.observations = observations;
	}

	public Mat getPrincipalDescriptor() {
		return principalDescriptor;
	}

	public void setPrincipalDescriptor(Mat principalDescriptor) {
		this.principalDescriptor = principalDescriptor;
	}

}

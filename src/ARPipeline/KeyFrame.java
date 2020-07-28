package ARPipeline;

import java.util.ArrayList;

import org.opencv.core.Mat;

public class KeyFrame {

	protected Pose pose = null;
	protected Mat descriptors = null;
	protected ArrayList<MapPoint> mapPoints = null;
	
	public KeyFrame() {
		
	}
	
	public KeyFrame(ArrayList<MapPoint> mapPoints) {
		this.mapPoints = mapPoints;
	}

	public Pose getPose() {
		return pose;
	}

	public void setPose(Pose pose) {
		this.pose = pose;
	}

	public Mat getDescriptors() {
		return descriptors;
	}

	public void setDescriptors(Mat descriptors) {
		this.descriptors = descriptors;
	}

	public ArrayList<MapPoint> getMapPoints() {
		return mapPoints;
	}

	public void setMapPoints(ArrayList<MapPoint> mapPoints) {
		this.mapPoints = mapPoints;
	}
	
}

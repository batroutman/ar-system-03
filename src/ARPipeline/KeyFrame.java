package ARPipeline;

import java.util.ArrayList;

import org.opencv.core.Mat;

public class KeyFrame {

	// pose with respect to origin/identity
	protected Pose pose = null;

	protected ArrayList<MapPoint> mapPoints = new ArrayList<MapPoint>();
	protected Mat descriptors = null;
	protected ArrayList<Point2D> keypoints = new ArrayList<Point2D>();

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

	public ArrayList<MapPoint> getMapPoints() {
		return mapPoints;
	}

	public void setMapPoints(ArrayList<MapPoint> mapPoints) {
		this.mapPoints = mapPoints;
	}

	public Mat getDescriptors() {
		return descriptors;
	}

	public void setDescriptors(Mat descriptors) {
		this.descriptors = descriptors;
	}

	public ArrayList<Point2D> getKeypoints() {
		return keypoints;
	}

	public void setKeypoints(ArrayList<Point2D> keypoints) {
		this.keypoints = keypoints;
	}

}

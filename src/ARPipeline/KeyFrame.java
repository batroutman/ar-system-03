package ARPipeline;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;

public class KeyFrame {

	protected Pose pose = null;
	protected Mat descriptors = null;
	protected ArrayList<MapPoint> mapPoints = new ArrayList<MapPoint>();
	protected List<Point> keypoints = new ArrayList<Point>();

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

	public List<Point> getKeypoints() {
		return keypoints;
	}

	public void setKeypoints(List<Point> keypoints) {
		this.keypoints = keypoints;
	}

	public void setKeypoints(MatOfKeyPoint keypoints) {
		List<KeyPoint> listKeypoints = keypoints.toList();
		List<Point> listPoints = new ArrayList<Point>();
		for (int i = 0; i < 100; i++) {
			listPoints.add(listKeypoints.get(i).pt);
		}
		this.keypoints = listPoints;
	}

}

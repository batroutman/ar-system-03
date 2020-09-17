package ARPipeline;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;

public class KeyFrame {

	// pose with respect to origin/identity
	protected Pose pose = null;

	protected Mat descriptors = null;
	protected ArrayList<KeyFrameEntry> entries = new ArrayList<KeyFrameEntry>();
	protected List<Point> keypoints = new ArrayList<Point>();

	public KeyFrame() {

	}

	public KeyFrame(ArrayList<KeyFrameEntry> entries) {
		this.entries = entries;
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

	public ArrayList<KeyFrameEntry> getEntries() {
		return this.entries;
	}

	public void setEntries(ArrayList<KeyFrameEntry> entries) {
		this.entries = entries;
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
		for (int i = 0; i < listKeypoints.size(); i++) {
			listPoints.add(listKeypoints.get(i).pt);
		}
		this.keypoints = listPoints;
	}

}

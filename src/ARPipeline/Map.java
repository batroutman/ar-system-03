package ARPipeline;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;

public class Map {

	protected ArrayList<MapPoint> determinedPoints = new ArrayList<MapPoint>();
	protected ArrayList<MapPoint> undeterminedPoints = new ArrayList<MapPoint>();
	protected Mat determinedDescriptors = null;
	protected Mat undeterminedDescriptors = null;
	protected boolean updateDescriptors = true;
	protected ArrayList<KeyFrame> keyframes = new ArrayList<KeyFrame>();

	public Map() {

	}

	public void add(MapPoint mp) {
		if (mp.getPoint() != null) {
			determinedPoints.add(mp);
		} else {
			undeterminedPoints.add(mp);
		}
		updateDescriptors = true;
	}

	public KeyFrame generateInitialKeyFrame(Mat descriptors, MatOfKeyPoint keypoints) {
		List<KeyPoint> keypointList = keypoints.toList();
		KeyFrame keyframe = new KeyFrame();
		keyframe.setPose(new Pose());
		keyframe.setDescriptors(descriptors);
		for (int i = 0; i < keypointList.size(); i++) {
			Point2D point = new Point2D(keypointList.get(i).pt.x, keypointList.get(i).pt.y);
			keyframe.getKeypoints().add(point);
			Observation observation = new Observation();
			observation.setDescriptor(descriptors.row(i));
			observation.setPoint(point);
			observation.setKeyframe(keyframe);
			MapPoint mp = new MapPoint();
			mp.setPrincipalDescriptor(descriptors.row(i));
			mp.getObservations().add(observation);
			this.undeterminedPoints.add(mp);
			keyframe.getMapPoints().add(mp);
		}
		this.keyframes.add(keyframe);
		return keyframe;
	}

	public ArrayList<MapPoint> getDeterminedPoints() {
		return determinedPoints;
	}

	public void setDeterminedPoints(ArrayList<MapPoint> determinedPoints) {
		this.determinedPoints = determinedPoints;
	}

	public ArrayList<MapPoint> getUndeterminedPoints() {
		return undeterminedPoints;
	}

	public void setUndeterminedPoints(ArrayList<MapPoint> undeterminedPoints) {
		this.undeterminedPoints = undeterminedPoints;
	}

	public Mat getDeterminedDescriptors() {
		return determinedDescriptors;
	}

	public void setDeterminedDescriptors(Mat determinedDescriptors) {
		this.determinedDescriptors = determinedDescriptors;
	}

	public Mat getUndeterminedDescriptors() {
		return undeterminedDescriptors;
	}

	public void setUndeterminedDescriptors(Mat undeterminedDescriptors) {
		this.undeterminedDescriptors = undeterminedDescriptors;
	}

	public boolean isUpdateDescriptors() {
		return updateDescriptors;
	}

	public void setUpdateDescriptors(boolean updateDescriptors) {
		this.updateDescriptors = updateDescriptors;
	}

	public ArrayList<KeyFrame> getKeyframes() {
		return keyframes;
	}

	public void setKeyframes(ArrayList<KeyFrame> keyframes) {
		this.keyframes = keyframes;
	}

}

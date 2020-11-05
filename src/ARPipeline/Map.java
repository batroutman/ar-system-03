package ARPipeline;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;

import Jama.Matrix;

public class Map {

	protected ArrayList<MapPoint> mapPoints = new ArrayList<MapPoint>();
	protected ArrayList<KeyFrame> keyframes = new ArrayList<KeyFrame>();

	public Map() {

	}

	public KeyFrame generateInitialKeyFrame(Frame frame, long frameNum) {
		// extract features across entire frame
		Mat descriptors = new Mat();
		MatOfKeyPoint keypoints = new MatOfKeyPoint();
		ARUtils.extractFeatures(frame, keypoints, descriptors);

		List<KeyPoint> keypointList = keypoints.toList();
		KeyFrame keyframe = new KeyFrame();
		keyframe.setFrameNumber(frameNum);
		Pose pose = new Pose();
		pose.setFixed(true);
		keyframe.setPose(pose);
		keyframe.setDescriptors(ARUtils.decomposeDescriptorMat(descriptors));
		for (int i = 0; i < keypointList.size(); i++) {
			// register 2D point location
			Point2D point = new Point2D(keypointList.get(i).pt.x, keypointList.get(i).pt.y);
			keyframe.getKeypoints().add(point);
			keyframe.getLastKeypointLocations().add(point);

			// register the corresponding descriptor and last time this was seen
			// (now)
			keyframe.getLastKeypointDescriptors().add(descriptors.row(i));
			keyframe.getLastFrameObserved().add(frameNum);

			// register official keyframe observation for this keypoint and
			// descriptor
			Observation observation = new Observation();
			observation.setDescriptor(descriptors.row(i));
			observation.setPoint(point);
			observation.setKeyframe(keyframe);

			// finally, create map point for this feature
			MapPoint mp = new MapPoint();
			mp.setPrincipalDescriptor(descriptors.row(i));
			mp.getObservations().add(observation);
			keyframe.getMapPoints().add(mp);
			keyframe.getObsvToMapPoint().put((int) point.getX() + "," + (int) point.getY(), mp);
			this.mapPoints.add(mp);
		}
		this.keyframes.add(keyframe);
		return keyframe;
	}

	public KeyFrame registerNewKeyframe(Frame frame, long frameNum, Pose currentPose,
			ArrayList<Correspondence2D2D> correspondences, ArrayList<MapPoint> existingMapPoints) {

		// extract features across entire frame
		Mat descriptors = new Mat();
		MatOfKeyPoint keypoints = new MatOfKeyPoint();
		ARUtils.extractFeatures(frame, keypoints, descriptors);

		List<KeyPoint> keypointList = keypoints.toList();
		KeyFrame keyframe = new KeyFrame();
		keyframe.setFrameNumber(frameNum);

		// set up new pose
		Pose pose = new Pose();
		pose.setFixed(false);
		pose.setCx(currentPose.getCx());
		pose.setCy(currentPose.getCy());
		pose.setCz(currentPose.getCz());
		pose.setQw(currentPose.getQw());
		pose.setQx(currentPose.getQx());
		pose.setQy(currentPose.getQy());
		pose.setQz(currentPose.getQz());
		keyframe.setPose(pose);

		// handle pre existing points
		for (int c = 0; c < correspondences.size(); c++) {
			Correspondence2D2D corr = correspondences.get(c);
			keyframe.getObsvToMapPoint().put(
					correspondences.get(c).getU2().intValue() + "," + correspondences.get(c).getV2().intValue(),
					existingMapPoints.get(c));
			keyframe.getDescriptors().add(corr.getDescriptor2());
			keyframe.getLastKeypointDescriptors().add(corr.getDescriptor2());
			Point2D keypointLocation = new Point2D(corr.getU2(), corr.getV2());
			keyframe.getKeypoints().add(keypointLocation);
			keyframe.getLastKeypointLocations().add(keypointLocation);
			keyframe.getLastFrameObserved().add(frameNum);

			// add observation
			Observation observation = new Observation();
			observation.setDescriptor(corr.getDescriptor2());
			observation.setPoint(keypointLocation);
			observation.setKeyframe(keyframe);
			// mp.setPrincipalDescriptor(descriptors.row(i));
			existingMapPoints.get(c).getObservations().add(observation);
			keyframe.getMapPoints().add(existingMapPoints.get(c));
			this.mapPoints.add(existingMapPoints.get(c));
		}

		// handle point that were just detected
		for (int i = 0; i < keypointList.size(); i++) {

			// check if map point is in hash table
			MapPoint mp = keyframe.getObsvToMapPoint()
					.get((int) keypointList.get(i).pt.x + "," + (int) keypointList.get(i).pt.y);
			if (mp == null) {
				mp = new MapPoint();
				keyframe.getObsvToMapPoint().put((int) keypointList.get(i).pt.x + "," + (int) keypointList.get(i).pt.y,
						mp);

				// add keypoint and descriptor to lists

				Point2D point = new Point2D(keypointList.get(i).pt.x, keypointList.get(i).pt.y);
				keyframe.getKeypoints().add(point);
				keyframe.getLastKeypointLocations().add(point);
				keyframe.getLastKeypointDescriptors().add(descriptors.row(i));
				keyframe.getDescriptors().add(descriptors.row(i));
				keyframe.getLastFrameObserved().add(frameNum);

				// add observation
				Observation observation = new Observation();
				observation.setDescriptor(descriptors.row(i));
				observation.setPoint(point);
				observation.setKeyframe(keyframe);
				// mp.setPrincipalDescriptor(descriptors.row(i));
				mp.getObservations().add(observation);
				keyframe.getMapPoints().add(mp);
				this.mapPoints.add(mp);
			}

		}
		this.keyframes.add(keyframe);
		return keyframe;
	}

	// given a camera pose, the camera intrinsics, and the width and height of
	// the frame, generate a visualization of the map's currently triangulated
	// map points
	public Frame getMapVisualizationFrame(Pose pose, Matrix K, int width, int height) {
		byte[] r = new byte[width * height];
		byte[] g = new byte[width * height];
		byte[] b = new byte[width * height];

		byte[][] colors = { { (byte) 255, 0, 0 }, { 0, (byte) 255, 0 }, { 0, 0, (byte) 255 },
				{ (byte) 255, (byte) 255, 0 }, { (byte) 255, 0, (byte) 255 }, { 0, (byte) 255, (byte) 255 } };

		for (int i = 0; i < width * height; i++) {
			r[i] = 0;
			g[i] = 0;
			b[i] = 0;
		}
		for (MapPoint mapPoint : this.mapPoints) {
			if (mapPoint.getPoint() == null) {
				continue;
			}
			Point3D point3D = mapPoint.getPoint();
			Matrix projectedPoint = K.times(pose.getHomogeneousMatrix().getMatrix(0, 2, 0, 3))
					.times(point3D.getHomogeneousPoint());
			double w = projectedPoint.get(2, 0);
			projectedPoint = projectedPoint.times(1 / w);
			// if (w < 0) {
			// System.out.println("w = 0 point projected behind camera");
			// }
			// if (projectedPoint.get(1, 0) < 0) {
			// System.out.println("CAMERA TOO LOW: " + projectedPoint.get(1,
			// 0));
			// }
			if (w > 0 && projectedPoint.get(0, 0) >= 0 && projectedPoint.get(0, 0) < width
					&& projectedPoint.get(1, 0) >= 0 && projectedPoint.get(1, 0) < height) {
				byte[] color = colors[Math.min(mapPoint.getObservations().size() - 1, colors.length)];
				r[(int) projectedPoint.get(1, 0) * width + (int) projectedPoint.get(0, 0)] = color[0];
				g[(int) projectedPoint.get(1, 0) * width + (int) projectedPoint.get(0, 0)] = color[1];
				b[(int) projectedPoint.get(1, 0) * width + (int) projectedPoint.get(0, 0)] = color[2];
			}
		}
		Frame frame = new Frame(r, g, b, width, height);
		return frame;
	}

	public ArrayList<KeyFrame> getKeyframes() {
		return keyframes;
	}

	public void setKeyframes(ArrayList<KeyFrame> keyframes) {
		this.keyframes = keyframes;
	}

	public ArrayList<MapPoint> getMapPoints() {
		return mapPoints;
	}

	public void setMapPoints(ArrayList<MapPoint> mapPoints) {
		this.mapPoints = mapPoints;
	}

}

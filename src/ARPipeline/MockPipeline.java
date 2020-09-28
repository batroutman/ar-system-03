package ARPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.features2d.FlannBasedMatcher;

import Jama.Matrix;

public class MockPipeline extends ARPipeline {

	MockPointData mock = new MockPointData();
	long frameNum = 0;
	boolean mapInitialized = false;

	Matrix K = new Matrix(3, 3);

	ArrayList<KeyFrame> keyframes = new ArrayList<KeyFrame>();
	ArrayList<KeyFrame> tempKeyframes = new ArrayList<KeyFrame>();
	KeyFrame currentKeyFrame = null;

	Pose pose = new Pose();

	protected Thread mainThread = new Thread() {
		@Override
		public void run() {
			mainloop();
		}
	};

	public MockPipeline() {
		super();
		this.init();
	}

	public MockPipeline(FrameBuffer inputFrameBuffer) {
		super();
		this.init();
		this.setInputFrameBuffer(inputFrameBuffer);
	}

	public MockPipeline(FrameBuffer inputFrameBuffer, PoseBuffer outputPoseBuffer) {
		super();
		this.init();
		this.setInputFrameBuffer(inputFrameBuffer);
		this.setOutputPoseBuffer(outputPoseBuffer);
	}

	public MockPipeline(FrameBuffer inputFrameBuffer, FrameBuffer outputFrameBuffer) {
		super();
		this.init();
		this.setInputFrameBuffer(inputFrameBuffer);
		this.setOutputFrameBuffer(outputFrameBuffer);
	}

	public MockPipeline(FrameBuffer inputFrameBuffer, PoseBuffer outputPoseBuffer, FrameBuffer outputFrameBuffer) {
		super();
		this.init();
		this.setInputFrameBuffer(inputFrameBuffer);
		this.setOutputPoseBuffer(outputPoseBuffer);
		this.setOutputFrameBuffer(outputFrameBuffer);
	}

	public void init() {
		K.set(0, 0, CameraIntrinsics.fx);
		K.set(0, 1, CameraIntrinsics.s);
		K.set(0, 2, CameraIntrinsics.cx);
		K.set(1, 0, 0.0);
		K.set(1, 1, CameraIntrinsics.fy);
		K.set(1, 2, CameraIntrinsics.cy);
		K.set(2, 0, 0.0);
		K.set(2, 1, 0.0);
		K.set(2, 2, 1.0);

	}

	public void start() {
		this.mainThread.start();
	}

	public void stop() {
		this.mainThread.interrupt();
	}

	public static KeyFrame generateKeyFrame(Mat descriptors, MatOfKeyPoint keypoints) {

		KeyFrame keyframe = new KeyFrame();
		Pose pose = new Pose();
		pose.setOrigin();
		keyframe.setPose(pose);
		keyframe.setDescriptors(descriptors);
		keyframe.setKeypoints(keypoints);

		ArrayList<KeyFrameEntry> entries = new ArrayList<KeyFrameEntry>();
		List<KeyPoint> keypointList = keypoints.toList();
		// may have issues here. negative y? offset?

		for (int i = 0; i < keypointList.size(); i++) {
			KeyFrameEntry entry = new KeyFrameEntry();
			entry.setDescriptor(descriptors.row(i));
			Point2D keypoint = new Point2D(keypointList.get(i).pt.x, keypointList.get(i).pt.y);
			entry.setKeypoint(keypoint);
			entries.add(entry);
		}

		keyframe.setEntries(entries);
		return keyframe;

	}

	public void updatePose(Matrix R, Matrix t) {

		Matrix E = Matrix.identity(4, 4);
		E.set(0, 0, R.get(0, 0));
		E.set(0, 1, R.get(0, 1));
		E.set(0, 2, R.get(0, 2));
		E.set(0, 3, t.get(0, 0));

		E.set(1, 0, R.get(1, 0));
		E.set(1, 1, R.get(1, 1));
		E.set(1, 2, R.get(1, 2));
		E.set(1, 3, t.get(1, 0));

		E.set(2, 0, R.get(2, 0));
		E.set(2, 1, R.get(2, 1));
		E.set(2, 2, R.get(2, 2));
		E.set(2, 3, t.get(2, 0));

		Matrix newPose = E.times(this.currentKeyFrame.getPose().getHomogeneousMatrix());

		this.pose.setMatrix(newPose);
		// System.out.println("updatedPose (true pose):");
		// newPose.print(5, 4);

	}

	public void setPose(Matrix R, Matrix t) {

		Matrix E = Matrix.identity(4, 4);
		E.set(0, 0, R.get(0, 0));
		E.set(0, 1, R.get(0, 1));
		E.set(0, 2, R.get(0, 2));
		E.set(0, 3, t.get(0, 0));

		E.set(1, 0, R.get(1, 0));
		E.set(1, 1, R.get(1, 1));
		E.set(1, 2, R.get(1, 2));
		E.set(1, 3, t.get(1, 0));

		E.set(2, 0, R.get(2, 0));
		E.set(2, 1, R.get(2, 1));
		E.set(2, 2, R.get(2, 2));
		E.set(2, 3, t.get(2, 0));

		this.pose.setMatrix(E);
		// System.out.println("updatedPose (true pose):");
		// newPose.print(5, 4);

	}

	public static void matchDescriptors(Mat descriptors, MatOfKeyPoint keypoints, KeyFrame currentKeyFrame,
			ArrayList<Correspondence2D2D> correspondences, ArrayList<Point> matchedKeyframePoints,
			ArrayList<Point> matchedPoints) {
		double DIST_THRESH = 150;
		FlannBasedMatcher flann = FlannBasedMatcher.create();
		ArrayList<MatOfDMatch> matches = new ArrayList<MatOfDMatch>();
		flann.knnMatch(descriptors, currentKeyFrame.getDescriptors(), matches, 1);
		for (int i = 0; i < matches.size(); i++) {
			DMatch match = matches.get(i).toList().get(0);
			if (match.distance < DIST_THRESH) {
				Correspondence2D2D c = new Correspondence2D2D();
				c.setU1(currentKeyFrame.getKeypoints().get(match.trainIdx).x);
				c.setV1(currentKeyFrame.getKeypoints().get(match.trainIdx).y);
				c.setU2(keypoints.toList().get(match.queryIdx).pt.x);
				c.setV2(keypoints.toList().get(match.queryIdx).pt.y);
				c.setDescriptor1(currentKeyFrame.getDescriptors().row(match.trainIdx));
				c.setDescriptor2(descriptors.row(match.queryIdx));
				correspondences.add(c);
				matchedKeyframePoints.add(currentKeyFrame.getKeypoints().get(match.trainIdx));
				matchedPoints.add(keypoints.toList().get(match.queryIdx).pt);
			}
		}
	}

	public void structureFromMotionUpdate(ArrayList<Point> matchedKeyframePoints, ArrayList<Point> matchedPoints,
			ArrayList<Correspondence2D2D> correspondences) {
		// compute fundamental matrix -> essential matrix -> [ R t ]
		MatOfPoint2f keyframeMat = new MatOfPoint2f();
		MatOfPoint2f matKeypoints = new MatOfPoint2f();
		keyframeMat.fromList(matchedKeyframePoints);
		matKeypoints.fromList(matchedPoints);
		Mat fundamentalMatrix = Calib3d.findFundamentalMat(keyframeMat, matKeypoints, Calib3d.FM_8POINT, 0.1, 0.99);

		Matrix funMat = new Matrix(3, 3);
		funMat.set(0, 0, fundamentalMatrix.get(0, 0)[0]);
		funMat.set(0, 1, fundamentalMatrix.get(0, 1)[0]);
		funMat.set(0, 2, fundamentalMatrix.get(0, 2)[0]);
		funMat.set(1, 0, fundamentalMatrix.get(1, 0)[0]);
		funMat.set(1, 1, fundamentalMatrix.get(1, 1)[0]);
		funMat.set(1, 2, fundamentalMatrix.get(1, 2)[0]);
		funMat.set(2, 0, fundamentalMatrix.get(2, 0)[0]);
		funMat.set(2, 1, fundamentalMatrix.get(2, 1)[0]);
		funMat.set(2, 2, fundamentalMatrix.get(2, 2)[0]);
		pl("estimated fundamental matrix:");
		funMat.print(15, 5);

		Matrix eMatrix = this.K.transpose().times(funMat).times(this.K);

		EssentialDecomposition decomp = ARUtils.decomposeEssentialMat(eMatrix);

		Rt rt = ARUtils.selectEssentialSolution(decomp, this.currentKeyFrame.getPose().getHomogeneousMatrix(),
				correspondences);
		Matrix R = this.mock.getR(this.frameNum);
		Matrix c = this.mock.getIC(this.frameNum).getMatrix(0, 2, 3, 3);

		Matrix t = R.getMatrix(0, 2, 0, 2).times(c);
		System.out.println("true R:");
		R.print(15, 5);
		System.out.println("true t:");
		t.print(15, 5);
		System.out.println("true t (unit):");
		t.times(1 / t.normF()).print(15, 5);
		System.out.println("true t (norm): " + t.normF());

		// test triangulation
		Matrix E = R.times(this.mock.getIC(this.frameNum));
		Matrix triangulation = ARUtils.triangulate(E, this.currentKeyFrame.getPose().getHomogeneousMatrix(),
				correspondences.get(0));
		pl("triangulation: ");
		triangulation.print(15, 5);

		pl("estimated t: ");
		rt.getT().print(15, 5);
		pl("estimated t (norm): " + rt.getT().normF());

		// MOCK PURPOSE
		this.updatePose(R, t);

		// TRUE PURPOSE
		// this.updatePose(rt.getR(), rt.getT());
	}

	// given a list of correspondences between the current keyframe and the
	// current frame, triangulate the 3D map points for keyframe entries in the
	// current keyframe using the current keyframe pose and the current system
	// pose (which must be updated before calling this function)
	public void triangulateMapPoints(ArrayList<Correspondence2D2D> correspondences) {
		for (int c = 0; c < correspondences.size(); c++) {
			Correspondence2D2D corr = correspondences.get(c);

			// isolate the keyframe entry to add a 3D point to
			KeyFrameEntry keyframeEntry = null;
			for (int k = 0; k < this.currentKeyFrame.getEntries().size() && keyframeEntry == null; k++) {
				if (this.currentKeyFrame.getEntries().get(k).getKeypoint().getX() == corr.getU1()
						&& this.currentKeyFrame.getEntries().get(k).getKeypoint().getY() == corr.getV1()) {
					keyframeEntry = this.currentKeyFrame.getEntries().get(k);
				}
			}
			Matrix E = this.pose.getHomogeneousMatrix();
			Matrix pointMatrix = ARUtils.triangulate(E, this.currentKeyFrame.getPose().getHomogeneousMatrix(), corr);

			Point3D point3D = new Point3D(pointMatrix.get(0, 0), pointMatrix.get(1, 0), pointMatrix.get(2, 0));
			keyframeEntry.setPoint(point3D);
		}
	}

	Mat oldDesc = new Mat();
	int count = 0;

	protected void mainloop() {
		boolean keepGoing = true;
		while (keepGoing) {

			if (this.frameNum >= this.mock.getMAX_FRAMES()) {
				keepGoing = false;
				continue;
			}

			// find ORB features (omit for mock)
			ArrayList<Point> pointsList = this.mock.getKeypoints(this.frameNum);
			ArrayList<KeyPoint> keypointsList = new ArrayList<KeyPoint>();
			for (int i = 0; i < pointsList.size(); i++) {
				KeyPoint keypoint = new KeyPoint();
				keypoint.pt = pointsList.get(i);
				keypointsList.add(keypoint);
			}
			MatOfKeyPoint keypoints = new MatOfKeyPoint();
			keypoints.fromList(keypointsList);
			Mat descriptors = this.mock.getDescriptors();

			// if no keyframes exist, generate one
			if (this.keyframes.size() == 0) {
				this.currentKeyFrame = generateKeyFrame(descriptors, keypoints);
				this.keyframes.add(this.currentKeyFrame);

			}
			// b. otherwise,
			else {
				pl("===============================    FRAME " + this.frameNum
						+ "   ===================================");

				// match descriptors to those in currentKeyframe
				ArrayList<Correspondence2D2D> correspondences = new ArrayList<Correspondence2D2D>();
				ArrayList<Point> matchedKeyframePoints = new ArrayList<Point>();
				ArrayList<Point> matchedPoints = new ArrayList<Point>();
				matchDescriptors(descriptors, keypoints, this.currentKeyFrame, correspondences, matchedKeyframePoints,
						matchedPoints);
				pl("num correspondences: " + correspondences.size());

				// initialize the map (for mock purposes)
				if (!mapInitialized) {
					this.structureFromMotionUpdate(matchedKeyframePoints, matchedPoints, correspondences);

					// triangulate points in map
					this.triangulateMapPoints(correspondences);

					mapInitialized = true;
				} else {

					/*
					 * // bundle adjust tests ArrayList<Pose> cameras = new
					 * ArrayList<Pose>(); ArrayList<Point3D> point3Ds = new
					 * ArrayList<Point3D>(); ArrayList<ArrayList<Point2D>>
					 * observations = new ArrayList<ArrayList<Point2D>>();
					 * 
					 * for (int i = 0; i < this.keyframes.size(); i++) {
					 * cameras.add(this.keyframes.get(i).getPose()); for (int j
					 * = 0; j < this.keyframes.get(i).getEntries().size(); j++)
					 * { point3Ds.add(this.keyframes.get(i).getEntries().get(j).
					 * getPoint()); ArrayList<Point2D> obs = new
					 * ArrayList<Point2D>(); for (int k = 0; k <
					 * this.keyframes.size() + 1; k++) { obs.add(null); } double
					 * x = this.mock.getKeypoints(this.frameNum).get(j).x;
					 * double y =
					 * this.mock.getKeypoints(this.frameNum).get(j).y; Point2D
					 * pt = new Point2D(x, y); obs.set(i,
					 * this.keyframes.get(i).getEntries().get(j).getKeypoint());
					 * obs.set(i + 1, pt); observations.add(obs); } }
					 * cameras.add(this.pose);
					 * 
					 * ARUtils.bundleAdjust(cameras, point3Ds, observations, 2);
					 */
					// Perform PnP to solve
					ArrayList<Point3D> points3D = new ArrayList<Point3D>();
					ArrayList<Point2D> points2D = new ArrayList<Point2D>();
					Random rand = new Random();
					for (int c = 0; c < correspondences.size(); c++) {
						Correspondence2D2D corr = correspondences.get(c);
						KeyFrameEntry keyframeEntry = null;
						for (int k = 0; k < this.currentKeyFrame.getEntries().size() && keyframeEntry == null; k++) {
							if (this.currentKeyFrame.getEntries().get(k).getKeypoint().getX() == corr.getU1()
									&& this.currentKeyFrame.getEntries().get(k).getKeypoint().getY() == corr.getV1()) {
								keyframeEntry = this.currentKeyFrame.getEntries().get(k);
							}
						}
						// points3D.add(keyframeEntry.getPoint());
						Point3D point3D = new Point3D();
						point3D.setX(this.mock.getWorldCoordinates().get(c).get(0, 0) + rand.nextDouble() * 10);
						point3D.setY(this.mock.getWorldCoordinates().get(c).get(1, 0) - rand.nextDouble() * 10);
						point3D.setZ(this.mock.getWorldCoordinates().get(c).get(2, 0) + rand.nextDouble() * 10);
						points3D.add(point3D);
						Point2D point2D = new Point2D(corr.getU2(), corr.getV2());
						points2D.add(point2D);
					}

					Matrix E = ARUtils.PnP(points3D, points2D);
					Matrix R1 = E.getMatrix(0, 2, 0, 2);
					Matrix t1 = E.getMatrix(0, 2, 3, 3);
					this.setPose(R1, t1);
					pl("true E: ");
					Matrix R = this.mock.getR(this.frameNum);
					Matrix IC = this.mock.getIC(this.frameNum);
					Matrix trueE = R.times(IC);
					trueE.print(15, 5);

				}

			}

			synchronized (this.outputPoseBuffer) {
				this.outputPoseBuffer.pushPose(pose);
			}

			// for demo, just push the unaltered frame along to the output
			// buffer
			synchronized (this.outputFrameBuffer) {
				int width = 480;
				int height = 270;

				Frame currentFrame = ARUtils.artificialKeypointFrame(this.mock.getKeypoints(this.frameNum), width,
						height);
				this.outputFrameBuffer.pushFrame(currentFrame);
			}

			this.frameNum++;

			try {
				Thread.sleep(10);
			} catch (Exception e) {

			}
		}
	}

	public void getStats(Mat desc) {
		for (int i = 0; i < desc.cols(); i++) {
			ArrayList<Double> column = new ArrayList<Double>();
			for (int j = 0; j < desc.rows(); j++) {
				column.add(desc.get(j, i)[0]);
			}
			System.out.println("====  " + i + "  ===============================");
			System.out.println("Min    : " + ARUtils.min(column));
			System.out.println("Max    : " + ARUtils.max(column));
			System.out.println("Average: " + ARUtils.mean(column));
			System.out.println("Std Dev: " + ARUtils.stdDev(column));
		}
	}

	public static void p(Object s) {
		System.out.print(s);
	}

	public static void pl(Object s) {
		System.out.println(s);
	}

}
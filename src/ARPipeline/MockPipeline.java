package ARPipeline;

import java.util.ArrayList;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
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
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.so.Quaternion_F64;

public class MockPipeline extends ARPipeline {

	MockPointData mock = new MockPointData();
	long frameNum = 0;
	boolean mapInitialized = false;

	Matrix K = new Matrix(3, 3);

	KeyFrame currentKeyFrame = null;

	Map map = new Map();

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

	public void updatePoseFromCurrent(Matrix E) {

		Matrix currentTransform = this.currentKeyFrame.getPose().getHomogeneousMatrix();
		Matrix newTransform = E.times(currentTransform);

		DMatrixRMaj R = new DMatrixRMaj(3, 3);
		R.add(0, 0, newTransform.get(0, 0));
		R.add(0, 1, newTransform.get(0, 1));
		R.add(0, 2, newTransform.get(0, 2));
		R.add(1, 0, newTransform.get(1, 0));
		R.add(1, 1, newTransform.get(1, 1));
		R.add(1, 2, newTransform.get(1, 2));
		R.add(2, 0, newTransform.get(2, 0));
		R.add(2, 1, newTransform.get(2, 1));
		R.add(2, 2, newTransform.get(2, 2));

		Quaternion_F64 q = ConvertRotation3D_F64.matrixToQuaternion(R, null);
		q.normalize();

		this.pose.setQw(q.w);
		this.pose.setQx(q.x);
		this.pose.setQy(q.y);
		this.pose.setQz(q.z);

		this.pose.setT(newTransform.get(0, 3), newTransform.get(1, 3), newTransform.get(2, 3));

	}

	public void setPose(Matrix E) {

		DMatrixRMaj R = new DMatrixRMaj(3, 3);
		R.add(0, 0, E.get(0, 0));
		R.add(0, 1, E.get(0, 1));
		R.add(0, 2, E.get(0, 2));
		R.add(1, 0, E.get(1, 0));
		R.add(1, 1, E.get(1, 1));
		R.add(1, 2, E.get(1, 2));
		R.add(2, 0, E.get(2, 0));
		R.add(2, 1, E.get(2, 1));
		R.add(2, 2, E.get(2, 2));

		Quaternion_F64 q = ConvertRotation3D_F64.matrixToQuaternion(R, null);
		q.normalize();

		this.pose.setQw(q.w);
		this.pose.setQx(q.x);
		this.pose.setQy(q.y);
		this.pose.setQz(q.z);

		this.pose.setT(E.get(0, 3), E.get(1, 3), E.get(2, 3));

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
				c.setU1(currentKeyFrame.getKeypoints().get(match.trainIdx).getX());
				c.setV1(currentKeyFrame.getKeypoints().get(match.trainIdx).getY());
				c.setU2(keypoints.toList().get(match.queryIdx).pt.x);
				c.setV2(keypoints.toList().get(match.queryIdx).pt.y);
				c.setDescriptor1(currentKeyFrame.getDescriptors().row(match.trainIdx));
				c.setDescriptor2(descriptors.row(match.queryIdx));
				correspondences.add(c);
				Point pt = new Point();
				pt.x = currentKeyFrame.getKeypoints().get(match.trainIdx).getX();
				pt.y = currentKeyFrame.getKeypoints().get(match.trainIdx).getY();
				matchedKeyframePoints.add(pt);
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

		// test triangulation
		Matrix E = R.times(this.mock.getIC(this.frameNum));

		// MOCK PURPOSE
		this.updatePoseFromCurrent(E);

		// TRUE PURPOSE
		// E.set(0, 0, rt.getR().get(0, 0));
		// E.set(0, 1, rt.getR().get(0, 1));
		// E.set(0, 2, rt.getR().get(0, 2));
		// E.set(1, 0, rt.getR().get(1, 0));
		// E.set(1, 1, rt.getR().get(1, 1));
		// E.set(1, 2, rt.getR().get(1, 2));
		// E.set(2, 0, rt.getR().get(2, 0));
		// E.set(2, 1, rt.getR().get(2, 1));
		// E.set(2, 2, rt.getR().get(2, 2));
		// E.set(0, 3, rt.getT().get(0, 0));
		// E.set(1, 3, rt.getT().get(1, 0));
		// E.set(2, 3, rt.getT().get(2, 0));
		// this.updatePoseFromCurrent(E);
	}

	// given a list of correspondences between the current keyframe and the
	// current frame, triangulate the 3D map points in the
	// current keyframe using the current keyframe pose and the current system
	// pose (which must be updated before calling this function)
	public void triangulateMapPoints(ArrayList<Correspondence2D2D> correspondences) {
		for (int c = 0; c < correspondences.size(); c++) {
			Correspondence2D2D corr = correspondences.get(c);

			// isolate the keyframe entry to add a 3D point to
			MapPoint mapPoint = null;
			for (int k = 0; k < this.currentKeyFrame.getMapPoints().size() && mapPoint == null; k++) {
				if (this.currentKeyFrame.getKeypoints().get(k).getX() == corr.getU1()
						&& this.currentKeyFrame.getKeypoints().get(k).getY() == corr.getV1()) {
					mapPoint = this.currentKeyFrame.getMapPoints().get(k);
				}
			}
			Matrix E = this.pose.getHomogeneousMatrix();
			Matrix pointMatrix = ARUtils.triangulate(E, this.currentKeyFrame.getPose().getHomogeneousMatrix(), corr);

			Point3D point3D = new Point3D(pointMatrix.get(0, 0), pointMatrix.get(1, 0), pointMatrix.get(2, 0));
			// mock
			// point3D.setX(this.mock.getWorldCoordinates().get(c).get(0, 0));
			// point3D.setY(this.mock.getWorldCoordinates().get(c).get(1, 0));
			// point3D.setZ(this.mock.getWorldCoordinates().get(c).get(2, 0));
			pl(point3D.getX() + ", " + point3D.getY() + ", " + point3D.getZ());
			mapPoint.setPoint(point3D);
		}
	}

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
			if (this.map.getKeyframes().size() == 0) {
				this.currentKeyFrame = this.map.generateInitialKeyFrame(descriptors, keypoints);
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

					// Perform PnP to solve
					ArrayList<Point3D> points3D = new ArrayList<Point3D>();
					ArrayList<Point2D> points2D = new ArrayList<Point2D>();
					Random rand = new Random();
					for (int c = 0; c < correspondences.size(); c++) {
						Correspondence2D2D corr = correspondences.get(c);
						MapPoint mapPoint = null;
						for (int k = 0; k < this.currentKeyFrame.getMapPoints().size() && mapPoint == null; k++) {
							if (this.currentKeyFrame.getKeypoints().get(k).getX() == corr.getU1()
									&& this.currentKeyFrame.getKeypoints().get(k).getY() == corr.getV1()) {
								mapPoint = this.currentKeyFrame.getMapPoints().get(k);
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
					this.setPose(E);
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
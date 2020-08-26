package ARPipeline;

import java.util.ArrayList;
import java.util.List;

import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector3f;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.features2d.FlannBasedMatcher;
import org.opencv.features2d.ORB;

import Jama.Matrix;

public class MockPipeline extends ARPipeline {

	Matrix K = new Matrix(3, 3);

	ArrayList<KeyFrame> keyframes = new ArrayList<KeyFrame>();
	KeyFrame currentKeyFrame = null;
	ArrayList<Correspondence> lastCorrespondences = null;

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

		ArrayList<MapPoint> mapPoints = new ArrayList<MapPoint>();
		List<KeyPoint> keypointList = keypoints.toList();
		// may have issues here. negative y? offset?

		for (int i = 0; i < keypointList.size(); i++) {
			MapPoint mp = new MapPoint(descriptors.row(i));
			mp.setUV(keypointList.get(i).pt.x, keypointList.get(i).pt.y);
			mapPoints.add(mp);
		}

		keyframe.setMapPoints(mapPoints);
		return keyframe;

	}

	public void updatePose(Matrix R, Matrix t) {

		Matrix4f updatedPose = new Matrix4f();
		updatedPose.setIdentity();
		updatedPose.m00 = (float) R.get(0, 0);
		updatedPose.m01 = (float) R.get(0, 1);
		updatedPose.m02 = (float) R.get(0, 2);
		updatedPose.m10 = (float) R.get(1, 0);
		updatedPose.m11 = (float) R.get(1, 1);
		updatedPose.m12 = (float) R.get(1, 2);
		updatedPose.m20 = (float) R.get(2, 0);
		updatedPose.m21 = (float) R.get(2, 1);
		updatedPose.m22 = (float) R.get(2, 2);
		updatedPose.translate(new Vector3f((float) t.get(0, 0), (float) t.get(1, 0), (float) t.get(2, 0)));

		Matrix4f.mul(updatedPose, this.currentKeyFrame.getPose().getHomogeneousMatrix4f(), updatedPose);
		this.pose.setMatrix(updatedPose);
	}

	Mat oldDesc = new Mat();
	int count = 0;

	protected void mainloop() {
		boolean keepGoing = true;
		while (keepGoing) {

			if (currentFrame == null) {
				keepGoing = false;

				continue;
			}

			// find ORB features
			int patchSize = 5;
			ORB orb = ORB.create(100);
			orb.setScoreType(ORB.FAST_SCORE);
			orb.setPatchSize(patchSize);
			orb.setNLevels(1);
			MatOfKeyPoint keypoints = new MatOfKeyPoint();
			Mat descriptors = new Mat();
			Mat image = ARUtils.frameToMat(currentFrame);
			orb.detectAndCompute(image, new Mat(), keypoints, descriptors);
			descriptors.convertTo(descriptors, CvType.CV_32F);
			ARUtils.boxHighlight(currentFrame, keypoints, patchSize);
			oldDesc = descriptors;

			// what do I want to do?
			// find correspondences between descriptors and mappoints in used in
			// current keyframe
			// a. if I have 0 keyframes, generate one and set pose to origin.
			// continue.
			if (this.keyframes.size() == 0) {
				this.currentKeyFrame = generateKeyFrame(descriptors, keypoints);
				this.keyframes.add(this.currentKeyFrame);
			}
			// b. otherwise,
			else {

				double DIST_THRESH = 150;
				FlannBasedMatcher flann = FlannBasedMatcher.create();
				ArrayList<MatOfDMatch> matches = new ArrayList<MatOfDMatch>();
				flann.knnMatch(descriptors, this.currentKeyFrame.getDescriptors(), matches, 1);
				ArrayList<Correspondence2D2D> correspondences = new ArrayList<Correspondence2D2D>();
				ArrayList<Point> matchedKeyframes = new ArrayList<Point>();
				ArrayList<Point> matchedPoints = new ArrayList<Point>();
				for (int i = 0; i < matches.size(); i++) {
					DMatch match = matches.get(i).toList().get(0);
					if (match.distance < DIST_THRESH) {
						Correspondence2D2D c = new Correspondence2D2D();
						c.setU1(this.currentKeyFrame.getKeypoints().get(match.trainIdx).x);
						c.setV1(this.currentKeyFrame.getKeypoints().get(match.trainIdx).y);
						c.setU2(keypoints.toList().get(match.queryIdx).pt.x);
						c.setV2(keypoints.toList().get(match.queryIdx).pt.y);
						c.setDescriptor1(this.currentKeyFrame.getDescriptors().row(match.trainIdx));
						c.setDescriptor2(descriptors.row(match.queryIdx));
						correspondences.add(c);
						matchedKeyframes.add(this.currentKeyFrame.getKeypoints().get(match.trainIdx));
						matchedPoints.add(keypoints.toList().get(match.queryIdx).pt);
					}
				}

				// compute fundamental matrix -> essential matrix -> [ R t ]
				MatOfPoint2f keyframeMat = new MatOfPoint2f();
				MatOfPoint2f matKeypoints = new MatOfPoint2f();
				keyframeMat.fromList(matchedKeyframes);
				matKeypoints.fromList(matchedPoints);
				// keyframeMat.fromList(this.currentKeyFrame.getKeypoints().subList(0,
				// min));
				// matKeypoints.fromList(points.subList(0, min));
				// Mat fundamentalMatrix =
				// Calib3d.findFundamentalMat(keyframeMat, matKeypoints,
				// Calib3d.FM_RANSAC, 10, 0.8);
				Mat fundamentalMatrix = Calib3d.findFundamentalMat(keyframeMat, matKeypoints, Calib3d.FM_8POINT);

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
				System.out.println("funMat");
				funMat.print(5, 4);
				Matrix eMatrix = this.K.transpose().times(funMat).times(this.K);
				System.out.println("eMatrix");
				eMatrix.print(5, 4);
				EssentialDecomposition decomp = ARUtils.decomposeEssentialMat(eMatrix);
				Rt rt = ARUtils.selectEssentialSolution(decomp, this.pose.getHomogeneousMatrix(), correspondences);
				Matrix R = Matrix.identity(3, 3);
				Matrix t = new Matrix(3, 1);
				this.updatePose(rt.getR(), rt.getT());

				// i. else (I don't have at least 50(?) correspondences), but I
				// have at least 10, then generate new keyframe and use the same
				// correspondences for fundamental matrix -> essential matrix ->
				// [ R t ]. set current keyframe to new keyframe. continue.

				// ii. else (<10 correspondences), check other keyframes for
				// keyframe with most correspondences. If >= 10 correspondences,
				// set as current keyframe and use the correspondences for sfm.
				// if < 40, generate new keyframe.
				// iii. else, tracking lost (handle this later)
			}

			// rotate cube as demo that pose can be modified and displayed
			// rotAngle += 0.002f;
			// translation += 0.006f;
			// Matrix R = Matrix.identity(3, 3);
			// R.set(1, 1, Math.cos(rotAngle));
			// R.set(2, 2, Math.cos(rotAngle));
			// R.set(1, 2, -Math.sin(rotAngle));
			// R.set(2, 1, Math.sin(rotAngle));
			// Matrix t = new Matrix(3,1);
			// t.set(0, 0, translation);
			// this.updatePose(R, t);
			// pose.setR11((float)Math.cos(rotAngle));
			// pose.setR22((float)Math.cos(rotAngle));
			// pose.setR12(-(float)Math.sin(rotAngle));
			// pose.setR21((float)Math.sin(rotAngle));
			//
			synchronized (this.outputPoseBuffer) {
				this.outputPoseBuffer.pushPose(pose);
			}

			// for demo, just push the unaltered frame along to the output
			// buffer
			synchronized (this.outputFrameBuffer) {
				this.outputFrameBuffer.pushFrame(currentFrame);
			}

			currentFrame = this.inputFrameBuffer.getCurrentFrame();

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

}
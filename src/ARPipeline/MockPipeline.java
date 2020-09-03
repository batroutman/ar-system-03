package ARPipeline;

import java.util.ArrayList;
import java.util.List;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;

import Jama.Matrix;

public class MockPipeline extends ARPipeline {

	MockPointData mock = new MockPointData();
	long frameNum = 0;

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

	Mat oldDesc = new Mat();
	int count = 0;

	protected void mainloop() {
		boolean keepGoing = true;
		while (keepGoing) {

			if (this.frameNum >= this.mock.getMAX_FRAMES() - 1) {
				keepGoing = false;
				continue;
			}

			// find ORB features (omit for mock)
			ArrayList<Point> keypoints = this.mock.getKeypoints(this.frameNum);

			// if no keyframes exist, generate one
			if (this.keyframes.size() == 0) {
				this.currentKeyFrame = new KeyFrame();
				this.currentKeyFrame.setPose(new Pose());
				this.currentKeyFrame.setKeypoints(keypoints);
				this.keyframes.add(this.currentKeyFrame);
			}
			// b. otherwise,
			else {

				// compute fundamental matrix -> essential matrix -> [ R t ]
				MatOfPoint2f keyframeMat = new MatOfPoint2f();
				MatOfPoint2f matKeypoints = new MatOfPoint2f();
				keyframeMat.fromList(this.currentKeyFrame.getKeypoints());
				matKeypoints.fromList(keypoints);
				Mat fundamentalMatrix = Calib3d.findFundamentalMat(keyframeMat, matKeypoints, Calib3d.FM_RANSAC, 0.1,
						0.99);

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

				Matrix eMatrix = this.K.transpose().times(funMat).times(this.K);
				// eMatrix = eMatrix.times(1 / eMatrix.get(2, 2));
				// eMatrix.print(5, 4);

				Matrix point1 = new Matrix(3, 1);
				Matrix point2 = new Matrix(3, 1);

				point1.set(0, 0, this.currentKeyFrame.getKeypoints().get(0).x);
				point1.set(1, 0, this.currentKeyFrame.getKeypoints().get(0).y);
				point1.set(2, 0, 1);

				point2.set(0, 0, keypoints.get(0).x);
				point2.set(1, 0, keypoints.get(0).y);
				point2.set(2, 0, 1);

				point1 = this.K.inverse().times(point1);
				point2 = this.K.inverse().times(point2);

				Matrix result = point2.transpose().times(eMatrix).times(point1);
				// result.print(5, 4);

				EssentialDecomposition decomp = ARUtils.decomposeEssentialMat(eMatrix);
				// Mat essentialMatrix = Calib3d.findEssentialMat(keyframeMat,
				// matKeypoints);
				Mat R1 = new Mat();
				Mat R2 = new Mat();
				Mat t0 = new Mat();
				Calib3d.decomposeEssentialMat(ARUtils.MatrixToMat(eMatrix), R1, R2, t0);
				// ARUtils.MatToMatrix(t0).print(5, 4);

				ArrayList<Correspondence2D2D> correspondences = new ArrayList<Correspondence2D2D>();
				for (int i = 0; i < this.currentKeyFrame.getKeypoints().size(); i++) {
					Correspondence2D2D c = new Correspondence2D2D(this.currentKeyFrame.getKeypoints().get(i).x,
							this.currentKeyFrame.getKeypoints().get(i).y, keypoints.get(i).x, keypoints.get(i).y);
					correspondences.add(c);
				}

				Rt rt = ARUtils.selectEssentialSolution(decomp, this.currentKeyFrame.getPose().getHomogeneousMatrix(),
						correspondences);
				Matrix R = this.mock.getR(this.frameNum);
				Matrix c = this.mock.getIC(this.frameNum).getMatrix(0, 2, 3, 3);
				// c.print(5, 4);
				Matrix t = R.getMatrix(0, 2, 0, 2).times(c);
				System.out.println("true t:");
				t.print(5, 4);
				System.out.println("true t (unit):");
				t.times(1 / t.normF()).print(5, 4);
				System.out.println("true t (norm): " + t.normF());
				// this.updatePose(R, t);
				this.updatePose(rt.getR(), rt.getT());

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
				Thread.sleep(100);
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

}
package ARPipeline;

import java.util.ArrayList;
import java.util.Random;

import org.lwjgl.util.vector.Vector3f;
import org.opencv.core.Mat;
import org.opencv.core.Point;

import Jama.Matrix;

public class MockPointData {

	protected int HEIGHT = 270;
	protected int WIDTH = 480;
	protected long MAX_FRAMES = 250;
	protected int NUM_POINTS = 1000;
	protected int START_FRAME = 0;
	protected int SEED = 1;
	protected Matrix K = new Matrix(3, 3);

	// Starting pose parameters
	// NOTE: translations should be negative (-C)
	protected Vector3f initialTranslation = new Vector3f(0f, 0f, 0f);
	protected double initialRotX = 0.0;
	protected double initialRotY = 0.0;
	protected double initialRotZ = 0.0;

	// Amount to update R and t by each frame
	// NOTE: translations should be negative (-C)
	protected Vector3f translationVelocity = new Vector3f(0.05f, 0f, -0.2f);
	// protected Vector3f translationVelocity = new Vector3f(5f, 3f, -10f);
	protected double rotX = 0.00;
	protected double rotY = -0.01;
	protected double rotZ = -0.000;
	// protected double rotX = 0.5;
	// protected double rotY = 0.2;
	// protected double rotZ = 1.4;

	// List of homogeneous column vectors (4x1) corresponding to world
	// coordinates
	protected ArrayList<Matrix> worldCoordinates = new ArrayList<Matrix>();

	// fake descriptors for the world points
	protected Mat descriptors = null;

	public MockPointData() {
		this.init();
	}

	protected void init() {
		this.initK();
		this.initWorldCoordinates();
		this.generateDescriptors();
	}

	protected void initK() {
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

	protected void initWorldCoordinates() {
		String output = "";
		double Z_SPAWN_MIN = -100;
		double Z_SPAWN_MAX = 100;
		double Y_SPAWN_MIN = -20;
		double Y_SPAWN_MAX = 20;
		double X_SPAWN_MIN = -30;
		double X_SPAWN_MAX = 30;

		double Z_RANGE = Z_SPAWN_MAX - Z_SPAWN_MIN;
		double Y_RANGE = Y_SPAWN_MAX - Y_SPAWN_MIN;
		double X_RANGE = X_SPAWN_MAX - X_SPAWN_MIN;

		Random random = new Random(this.SEED);
		for (int i = 0; i < this.NUM_POINTS; i++) {
			Matrix point = new Matrix(4, 1);
			point.set(0, 0, random.nextDouble() * X_RANGE + X_SPAWN_MIN);
			point.set(1, 0, random.nextDouble() * Y_RANGE + Y_SPAWN_MIN);
			point.set(2, 0, random.nextDouble() * Z_RANGE + Z_SPAWN_MIN);
			point.set(3, 0, 1);
			this.worldCoordinates.add(point);
			output += point.get(0, 0) + ", " + point.get(1, 0) + ", " + point.get(2, 0) + "\n";
		}
		// System.out.println("true world coords:");
		// System.out.println(output);

	}

	// Returns homogeneous 4x4 matrix of JUST rotation parameters
	public Matrix getR(long frameNumber) {

		// Calculate this frame's rotation parameters
		float gamma = (float) this.rotX * (frameNumber + this.START_FRAME);
		float beta = (float) this.rotY * (frameNumber + this.START_FRAME);
		float alpha = (float) this.rotZ * (frameNumber + this.START_FRAME);

		Matrix Rx = Matrix.identity(4, 4);
		Rx.set(1, 1, Math.cos(gamma));
		Rx.set(2, 2, Math.cos(gamma));
		Rx.set(1, 2, -Math.sin(gamma));
		Rx.set(2, 1, Math.sin(gamma));

		Matrix Ry = Matrix.identity(4, 4);
		Ry.set(0, 0, Math.cos(beta));
		Ry.set(2, 2, Math.cos(beta));
		Ry.set(2, 0, -Math.sin(beta));
		Ry.set(0, 2, Math.sin(beta));

		Matrix Rz = Matrix.identity(4, 4);
		Rz.set(0, 0, Math.cos(alpha));
		Rz.set(1, 1, Math.cos(alpha));
		Rz.set(0, 1, -Math.sin(alpha));
		Rz.set(1, 0, Math.sin(alpha));

		return Rz.times(Ry).times(Rx);
	}

	// Returns homogeneous 4x4 matrix of WORLD translation parameters
	public Matrix getIC(long frameNumber) {
		// Calculate C
		Matrix C = Matrix.identity(4, 4);

		C.set(0, 3, this.initialTranslation.x + this.translationVelocity.x * (frameNumber + this.START_FRAME));
		C.set(1, 3, this.initialTranslation.y + this.translationVelocity.y * (frameNumber + this.START_FRAME));
		C.set(2, 3, this.initialTranslation.z + this.translationVelocity.z * (frameNumber + this.START_FRAME));

		return C;
	}

	public Matrix getQuaternion(long frameNumber) {

		// Calculate this frame's rotation parameters
		float gamma = (float) this.rotX * (frameNumber + this.START_FRAME);
		float beta = (float) this.rotY * (frameNumber + this.START_FRAME);
		float alpha = (float) this.rotZ * (frameNumber + this.START_FRAME);

		double cx = Math.cos(gamma * 0.5);
		double cy = Math.cos(beta * 0.5);
		double cz = Math.cos(alpha * 0.5);
		double sx = Math.sin(gamma * 0.5);
		double sy = Math.sin(beta * 0.5);
		double sz = Math.sin(alpha * 0.5);

		double qw = cx * cy * cz + sx * sy * sz;
		double qx = sx * cy * cz - cx * sy * sz;
		double qy = cx * sy * cz + sx * cy * sz;
		double qz = cx * cy * sz - sx * sy * cz;

		Matrix q = new Matrix(4, 1);
		q.set(0, 0, qw);
		q.set(1, 0, qx);
		q.set(2, 0, qy);
		q.set(3, 0, qz);

		q = q.times(1 / q.normF());

		return q;

	}

	public Mat getKeypointsAndDescriptors(long frameNumber, ArrayList<Point> outKeyPoints) {
		outKeyPoints.clear();

		Matrix R = this.getR(frameNumber);
		Matrix C = this.getIC(frameNumber);
		Matrix E = R.times(C);

		Matrix cameraMatrix = this.K.times(E.getMatrix(0, 2, 0, 3));

		ArrayList<Integer> validPoints = new ArrayList<Integer>();

		for (int i = 0; i < this.worldCoordinates.size(); i++) {
			// initial projection calculation
			Matrix point = cameraMatrix.times(this.worldCoordinates.get(i));
			double w = point.get(2, 0);

			// homogenize
			if (w != 0) {
				point = point.times(1 / w);
			}

			// check if point is in front of camera and is projected onto the
			// limited screen space
			if (w > 0 && point.get(0, 0) >= 0 && point.get(0, 0) < this.WIDTH && point.get(1, 0) >= 0
					&& point.get(1, 0) < this.HEIGHT) {
				Point pt = new Point(Math.round(point.get(0, 0)), Math.round(point.get(1, 0)));
				outKeyPoints.add(pt);
				validPoints.add(i);
				if (pt.x == 404 && pt.y == 231) {
					this.worldCoordinates.get(i).print(15, 5);
				}
			}

		}

		double avgNoiseRange = 0.0;
		double extremeNoiseRange = 0.0;
		double proportionBroken = 0.0;
		Mat descriptors = this.createDescriptors(frameNumber, validPoints, avgNoiseRange, extremeNoiseRange,
				proportionBroken);

		return descriptors;
	}

	public Mat createDescriptors(long frameNumber, ArrayList<Integer> validPoints, double avgNoiseRange,
			double extremeNoiseRange, double proportionBroken) {

		Random rand1 = new Random(frameNumber + 102);
		Random rand2 = new Random(frameNumber + 103);

		Matrix trueDescriptorMatrix = ARUtils.MatToMatrix(this.descriptors);
		Matrix newDescriptorMatrix = new Matrix(validPoints.size(), 32);

		for (int row = 0; row < validPoints.size(); row++) {

			double noiseRange = rand1.nextDouble() < proportionBroken ? extremeNoiseRange : avgNoiseRange;
			double offset = noiseRange / 2;

			for (int col = 0; col < 32; col++) {

				int newValue = (int) Math.round(trueDescriptorMatrix.get(validPoints.get(row), col)
						+ (rand2.nextDouble() * noiseRange - offset));
				newValue = newValue > 255 ? 255 : newValue;
				newValue = newValue < 0 ? 0 : newValue;
				newDescriptorMatrix.set(row, col, newValue);

			}
		}

		return ARUtils.MatrixToMat(newDescriptorMatrix);

	}

	public void generateDescriptors() {

		Random rand = new Random(101);

		Matrix desc = new Matrix(this.NUM_POINTS, 32);
		for (int i = 0; i < desc.getRowDimension(); i++) {
			for (int j = 0; j < desc.getColumnDimension(); j++) {
				desc.set(i, j, Math.floor(rand.nextDouble() * 256));
			}
		}

		this.descriptors = ARUtils.MatrixToMat(desc);

	}

	public ArrayList<Matrix> getWorldCoordinates() {
		return worldCoordinates;
	}

	public void setWorldCoordinates(ArrayList<Matrix> worldCoordinates) {
		this.worldCoordinates = worldCoordinates;
	}

	public int getHEIGHT() {
		return HEIGHT;
	}

	public void setHEIGHT(int hEIGHT) {
		HEIGHT = hEIGHT;
	}

	public int getWIDTH() {
		return WIDTH;
	}

	public void setWIDTH(int wIDTH) {
		WIDTH = wIDTH;
	}

	public long getMAX_FRAMES() {
		return MAX_FRAMES;
	}

	public void setMAX_FRAMES(long mAX_FRAMES) {
		MAX_FRAMES = mAX_FRAMES;
	}

	public int getNUM_POINTS() {
		return NUM_POINTS;
	}

	public void setNUM_POINTS(int nUM_POINTS) {
		NUM_POINTS = nUM_POINTS;
	}

	public Matrix getK() {
		return K;
	}

	public void setK(Matrix k) {
		K = k;
	}

	public Vector3f getInitialTranslation() {
		return initialTranslation;
	}

	public void setInitialTranslation(Vector3f initialTranslation) {
		this.initialTranslation = initialTranslation;
	}

	public double getInitialRotX() {
		return initialRotX;
	}

	public void setInitialRotX(double initialRotX) {
		this.initialRotX = initialRotX;
	}

	public double getInitialRotY() {
		return initialRotY;
	}

	public void setInitialRotY(double initialRotY) {
		this.initialRotY = initialRotY;
	}

	public double getInitialRotZ() {
		return initialRotZ;
	}

	public void setInitialRotZ(double initialRotZ) {
		this.initialRotZ = initialRotZ;
	}

	public Vector3f getTranslationVelocity() {
		return translationVelocity;
	}

	public void setTranslationVelocity(Vector3f translationVelocity) {
		this.translationVelocity = translationVelocity;
	}

	public double getRotX() {
		return rotX;
	}

	public void setRotX(double rotX) {
		this.rotX = rotX;
	}

	public double getRotY() {
		return rotY;
	}

	public void setRotY(double rotY) {
		this.rotY = rotY;
	}

	public double getRotZ() {
		return rotZ;
	}

	public void setRotZ(double rotZ) {
		this.rotZ = rotZ;
	}

	public Mat getDescriptors() {
		return descriptors;
	}

	public void setDescriptors(Mat descriptors) {
		this.descriptors = descriptors;
	}
}

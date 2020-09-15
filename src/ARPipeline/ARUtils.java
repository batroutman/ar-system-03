package ARPipeline;

import java.util.ArrayList;
import java.util.List;

import org.lwjgl.util.vector.Matrix4f;
import org.opencv.core.CvType;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;

import Jama.Matrix;
import Jama.SingularValueDecomposition;

public class ARUtils {

	/// E1 - 4x4 or 3x4 matrix of camera pose A
	/// E2 - 4x4 or 3x4 matrix of transformation from A to B with unit
	/// translation vector
	/// c - a correspondence between A and B used to estimate scale
	public static double estimateScale(Matrix E1, Matrix E2, Correspondence2D2D c) {
		Matrix K = CameraIntrinsics.getK4x4();

		double oldScore = 0;
		double THRESHOLD = 0.01;
		double scale = 1.0;
		double scaleJump = 10000;
		boolean goRight = true;
		boolean firstPass = true;
		boolean keepGoing = true;
		int i = 0;

		while (keepGoing) {

			// generate fundamental matrix between poses with some scale
			// // - calculate C = R1Inv.times(E1.getMatrix(0,2,3,3)).times(-1)
			Matrix R1Inv = E1.getMatrix(0, 2, 0, 2).inverse();
			Matrix C = R1Inv.times(E1.getMatrix(0, 2, 3, 3)).times(-1);
			Matrix Ch = new Matrix(4, 1);
			Ch.set(0, 0, C.get(0, 0));
			Ch.set(1, 0, C.get(1, 0));
			Ch.set(2, 0, C.get(2, 0));
			Ch.set(3, 0, 1);

			// // - E2 = [R2 | scale * t]
			Matrix E2scaled = E2.copy();
			E2scaled.set(0, 3, E2.get(0, 3) * scale);
			E2scaled.set(1, 3, E2.get(1, 3) * scale);
			E2scaled.set(2, 3, E2.get(2, 3) * scale);

			// // - calculate e' = E2.times(C)
			Matrix Pprime = K.getMatrix(0, 2, 0, 2).times(E2scaled.getMatrix(0, 2, 0, 3));
			Matrix e = Pprime.times(Ch);

			// // - generate [e']x
			Matrix ex = new Matrix(3, 3);
			ex.set(0, 1, -e.get(2, 0));
			ex.set(0, 2, e.get(1, 0));
			ex.set(1, 0, e.get(2, 0));
			ex.set(1, 2, -e.get(0, 0));
			ex.set(2, 0, -e.get(1, 0));
			ex.set(2, 1, e.get(0, 0));

			// // - calculate P+ = E1.inverse()
			Matrix P = K.getMatrix(0, 2, 0, 2).times(E1.getMatrix(0, 2, 0, 3));
			SingularValueDecomposition svd = P.transpose().svd();
			Matrix Splus = new Matrix(3, 3);
			Splus.set(0, 0, 1 / svd.getS().get(0, 0));
			Splus.set(1, 1, 1 / svd.getS().get(1, 1));
			Splus.set(2, 2, 1 / svd.getS().get(2, 2));
			Matrix U = svd.getV();
			Matrix V = svd.getU();
			Matrix PInv = V.times(Splus).times(U.transpose());

			// // - F = [e']x*(P'*P+)
			Matrix F = ex.times(Pprime.times(PInv));
			F = F.times(1 / F.get(2, 2));

			// test a correspondence
			Matrix x = new Matrix(3, 1);
			x.set(0, 0, c.getU1());
			x.set(1, 0, c.getV1());
			x.set(2, 0, 1);

			Matrix xprime = new Matrix(3, 1);
			xprime.set(0, 0, c.getU2());
			xprime.set(1, 0, c.getV2());
			xprime.set(2, 0, 1);

			// score = | x2' * F * x1 |
			double score = Math.abs(xprime.transpose().times(F).times(x).get(0, 0));

			// if the score is good enough, stop
			if (score < THRESHOLD || i >= 100) {
				keepGoing = false;
				continue;
			}

			// if this is the first pass, don't compare with old score
			if (firstPass) {
				firstPass = false;
				oldScore = score;
				continue;
			}

			// if this pose is worse than the last, flip search direction and
			// halve jump distance
			if (score > oldScore) {
				goRight = !goRight;
				if (scaleJump >= 0.25) {
					scaleJump = scaleJump / 2;
				}
			}
			// pl("score: " + score);
			// pl("scale: " + scale);
			// pl("scaleJump: " + scaleJump);

			// prepare a new scale for the next iteration
			scale = goRight ? scale + scaleJump : scale - scaleJump;
			oldScore = score;
			i++;

		}
		return scale;
	}

	public static Frame artificialKeypointFrame(List<Point> points, int width, int height) {
		byte[] r = new byte[width * height];
		byte[] g = new byte[width * height];
		byte[] b = new byte[width * height];
		for (int i = 0; i < width * height; i++) {
			r[i] = 0;
			g[i] = 0;
			b[i] = 0;
		}
		for (Point keypoint : points) {
			if (keypoint.x >= 0 && keypoint.x < width && keypoint.y >= 0 && keypoint.y < height) {
				r[(int) (keypoint.y * width + keypoint.x)] = (byte) 255;
				g[(int) (keypoint.y * width + keypoint.x)] = (byte) 255;
				b[(int) (keypoint.y * width + keypoint.x)] = (byte) 255;
			}
		}
		Frame frame = new Frame(r, g, b, width, height);
		return frame;
	}

	public static Matrix Matrix4fToMatrix(Matrix4f matrix4f) {
		Matrix matrix = new Matrix(4, 4);
		matrix.set(0, 0, matrix4f.m00);
		matrix.set(0, 1, matrix4f.m01);
		matrix.set(0, 2, matrix4f.m02);
		matrix.set(0, 3, matrix4f.m03);
		matrix.set(1, 0, matrix4f.m10);
		matrix.set(1, 1, matrix4f.m11);
		matrix.set(1, 2, matrix4f.m12);
		matrix.set(1, 3, matrix4f.m13);
		matrix.set(2, 0, matrix4f.m20);
		matrix.set(2, 1, matrix4f.m21);
		matrix.set(2, 2, matrix4f.m22);
		matrix.set(2, 3, matrix4f.m23);
		matrix.set(3, 0, matrix4f.m30);
		matrix.set(3, 1, matrix4f.m31);
		matrix.set(3, 2, matrix4f.m32);
		matrix.set(3, 3, matrix4f.m33);
		return matrix;
	}

	public static Matrix4f MatrixToMatrix4f(Matrix matrix) {
		Matrix4f matrix4f = new Matrix4f();
		matrix4f.m00 = (float) matrix.get(0, 0);
		matrix4f.m01 = (float) matrix.get(0, 1);
		matrix4f.m02 = (float) matrix.get(0, 2);
		matrix4f.m03 = (float) matrix.get(0, 3);
		matrix4f.m10 = (float) matrix.get(1, 0);
		matrix4f.m11 = (float) matrix.get(1, 1);
		matrix4f.m12 = (float) matrix.get(1, 2);
		matrix4f.m13 = (float) matrix.get(1, 3);
		matrix4f.m20 = (float) matrix.get(2, 0);
		matrix4f.m21 = (float) matrix.get(2, 1);
		matrix4f.m22 = (float) matrix.get(2, 2);
		matrix4f.m23 = (float) matrix.get(2, 3);
		matrix4f.m30 = (float) matrix.get(3, 0);
		matrix4f.m31 = (float) matrix.get(3, 1);
		matrix4f.m32 = (float) matrix.get(3, 2);
		matrix4f.m33 = (float) matrix.get(3, 3);
		return matrix4f;
	}

	public static ArrayList<Correspondence2D2D> pruneCorrespondences(Matrix fundamentalMatrix,
			ArrayList<Correspondence2D2D> correspondences) {
		ArrayList<Correspondence2D2D> newC = new ArrayList<Correspondence2D2D>();
		for (int i = 0; i < correspondences.size(); i++) {
			Matrix x1 = new Matrix(3, 1);
			x1.set(0, 0, correspondences.get(i).getU1());
			x1.set(1, 0, correspondences.get(i).getV1());
			x1.set(2, 0, 1);

			Matrix x2 = new Matrix(3, 1);
			x2.set(0, 0, correspondences.get(i).getU2());
			x2.set(1, 0, correspondences.get(i).getV2());
			x2.set(2, 0, 1);

			double EPI_THRESH = 0.2;
			double epipolar = x2.transpose().times(fundamentalMatrix).times(x1).get(0, 0);
			// System.out.println("epipolar: " + epipolar);
			if (Math.abs(epipolar) < EPI_THRESH) {
				newC.add(correspondences.get(i));
			}

		}
		return newC;
	}

	public static Matrix triangulate(Matrix E, Matrix pose, Correspondence2D2D c) {

		Matrix Pprime = E.times(pose);
		Pprime = CameraIntrinsics.getK4x4().times(Pprime);

		Matrix P = CameraIntrinsics.getK4x4().times(pose);

		// compute A matrix for Ax = 0
		Matrix row0 = P.getMatrix(2, 2, 0, 3).times(c.getU1()).minus(P.getMatrix(0, 0, 0, 3));
		Matrix row1 = P.getMatrix(2, 2, 0, 3).times(c.getV1()).minus(P.getMatrix(1, 1, 0, 3));
		Matrix row2 = Pprime.getMatrix(2, 2, 0, 3).times(c.getU2()).minus(Pprime.getMatrix(0, 0, 0, 3));
		Matrix row3 = Pprime.getMatrix(2, 2, 0, 3).times(c.getV2()).minus(Pprime.getMatrix(1, 1, 0, 3));

		Matrix A = new Matrix(4, 4);
		A.set(0, 0, row0.get(0, 0));
		A.set(0, 1, row0.get(0, 1));
		A.set(0, 2, row0.get(0, 2));
		A.set(0, 3, row0.get(0, 3));
		A.set(1, 0, row1.get(0, 0));
		A.set(1, 1, row1.get(0, 1));
		A.set(1, 2, row1.get(0, 2));
		A.set(1, 3, row1.get(0, 3));
		A.set(2, 0, row2.get(0, 0));
		A.set(2, 1, row2.get(0, 1));
		A.set(2, 2, row2.get(0, 2));
		A.set(2, 3, row2.get(0, 3));
		A.set(3, 0, row3.get(0, 0));
		A.set(3, 1, row3.get(0, 1));
		A.set(3, 2, row3.get(0, 2));
		A.set(3, 3, row3.get(0, 3));

		SingularValueDecomposition svd = A.svd();
		Matrix X = svd.getV().getMatrix(0, 3, 3, 3);
		X = X.times(1.0 / X.get(3, 0));
		// System.out.println("X");
		// X.print(5, 4);
		return X;
	}

	public static Rt selectEssentialSolution(EssentialDecomposition decomp, Matrix pose,
			ArrayList<Correspondence2D2D> correspondences) {
		Rt rt = new Rt();
		rt.setR(decomp.getR1());
		rt.setT(decomp.getT1());

		// get scale for each combination
		double scale = 18.1384;

		// set up extrinsic matrices (both possible options)
		Matrix E1 = Matrix.identity(4, 4);
		Matrix E2 = Matrix.identity(4, 4);
		Matrix E3 = Matrix.identity(4, 4);
		Matrix E4 = Matrix.identity(4, 4);
		E1.set(0, 0, decomp.getR1().get(0, 0));
		E1.set(0, 1, decomp.getR1().get(0, 1));
		E1.set(0, 2, decomp.getR1().get(0, 2));
		E1.set(1, 0, decomp.getR1().get(1, 0));
		E1.set(1, 1, decomp.getR1().get(1, 1));
		E1.set(1, 2, decomp.getR1().get(1, 2));
		E1.set(2, 0, decomp.getR1().get(2, 0));
		E1.set(2, 1, decomp.getR1().get(2, 1));
		E1.set(2, 2, decomp.getR1().get(2, 2));
		E1.set(0, 3, decomp.getT1().get(0, 0) * scale);
		E1.set(1, 3, decomp.getT1().get(1, 0) * scale);
		E1.set(2, 3, decomp.getT1().get(2, 0) * scale);

		E2.set(0, 0, decomp.getR2().get(0, 0));
		E2.set(0, 1, decomp.getR2().get(0, 1));
		E2.set(0, 2, decomp.getR2().get(0, 2));
		E2.set(1, 0, decomp.getR2().get(1, 0));
		E2.set(1, 1, decomp.getR2().get(1, 1));
		E2.set(1, 2, decomp.getR2().get(1, 2));
		E2.set(2, 0, decomp.getR2().get(2, 0));
		E2.set(2, 1, decomp.getR2().get(2, 1));
		E2.set(2, 2, decomp.getR2().get(2, 2));
		E2.set(0, 3, decomp.getT1().get(0, 0) * scale);
		E2.set(1, 3, decomp.getT1().get(1, 0) * scale);
		E2.set(2, 3, decomp.getT1().get(2, 0) * scale);

		E3 = E1.copy();
		E3.set(0, 3, -E3.get(0, 3));
		E3.set(1, 3, -E3.get(1, 3));
		E3.set(2, 3, -E3.get(2, 3));

		E4 = E2.copy();
		E4.set(0, 3, -E4.get(0, 3));
		E4.set(1, 3, -E4.get(1, 3));
		E4.set(2, 3, -E4.get(2, 3));

		// pick a correspondence
		Correspondence2D2D c = correspondences.get(0);

		Matrix X1 = triangulate(E1, pose, c);
		Matrix X2 = triangulate(E2, pose, c);
		Matrix X3 = triangulate(E3, pose, c);
		Matrix X4 = triangulate(E4, pose, c);

		pl("X1: ");
		X1.print(10, 5);
		pl("X2: ");
		X2.print(10, 5);
		pl("X3: ");
		X3.print(10, 5);
		pl("X4: ");
		X4.print(10, 5);

		// fix this
		Matrix b1 = CameraIntrinsics.getK4x4().times(E1).times(pose).times(X1);
		Matrix b2 = CameraIntrinsics.getK4x4().times(E2).times(pose).times(X2);
		Matrix b3 = CameraIntrinsics.getK4x4().times(E3).times(pose).times(X3);
		Matrix b4 = CameraIntrinsics.getK4x4().times(E4).times(pose).times(X4);
		int numSet = 0;

		if (b1.get(2, 0) > 0) {
			Matrix a1 = pose.times(X1);
			if (a1.get(2, 0) > 0) {
				rt.setR(decomp.getR1());
				rt.setT(decomp.getT1().times(scale));
				numSet++;
			}
		}

		if (b2.get(2, 0) > 0) {
			Matrix a2 = pose.times(X2);
			if (a2.get(2, 0) > 0) {
				rt.setR(decomp.getR2());
				rt.setT(decomp.getT1().times(scale));
				numSet++;
			}
		}

		if (b3.get(2, 0) > 0) {
			Matrix a3 = pose.times(X3);
			if (a3.get(2, 0) > 0) {
				rt.setR(decomp.getR1());
				rt.setT(decomp.getT1().times(scale).times(-1));
				numSet++;
			}
		}

		if (b4.get(2, 0) > 0) {
			Matrix a4 = pose.times(X3);
			if (a4.get(2, 0) > 0) {
				rt.setR(decomp.getR2());
				rt.setT(decomp.getT1().times(scale).times(-1));
				numSet++;
			}
		}

		pl("numSet = " + numSet);

		pl("chosen R: ");
		rt.getR().print(15, 5);
		pl("chosen t: ");
		rt.getT().print(15, 5);

		return rt;
	}

	public static Matrix transformPose(Matrix R, Matrix t, Matrix pose) {
		// make homogeneous
		Matrix homogeneousE = Matrix.identity(4, 4);
		homogeneousE.set(0, 0, R.get(0, 0));
		homogeneousE.set(0, 1, R.get(0, 1));
		homogeneousE.set(0, 2, R.get(0, 2));
		homogeneousE.set(1, 0, R.get(1, 0));
		homogeneousE.set(1, 1, R.get(1, 1));
		homogeneousE.set(1, 2, R.get(1, 2));
		homogeneousE.set(2, 0, R.get(2, 0));
		homogeneousE.set(2, 1, R.get(2, 1));
		homogeneousE.set(2, 2, R.get(2, 2));
		homogeneousE.set(0, 3, t.get(0, 0));
		homogeneousE.set(1, 3, t.get(1, 0));
		homogeneousE.set(2, 3, t.get(2, 0));

		Matrix newPose = homogeneousE.times(pose);
		return newPose;
	}

	public static EssentialDecomposition decomposeEssentialMat(Matrix essentialMatrix) {
		EssentialDecomposition decomp = new EssentialDecomposition();
		SingularValueDecomposition svd = essentialMatrix.svd();

		Matrix W = new Matrix(3, 3);
		W.set(0, 1, -1);
		W.set(1, 0, 1);
		W.set(2, 2, 1);

		Matrix Z = new Matrix(3, 3);
		Z.set(0, 1, 1);
		Z.set(1, 0, -1);

		Matrix t1 = svd.getU().getMatrix(0, 2, 2, 2);
		Matrix t2 = t1.times(-1);
		Matrix R1 = svd.getU().times(W.transpose()).times(svd.getV().transpose());
		Matrix R2 = svd.getU().times(W).times(svd.getV().transpose());

		decomp.setR1(R1);
		decomp.setR2(R2);
		decomp.setT1(t1);
		decomp.setT2(t2);

		pl("R1: ");
		R1.print(15, 5);
		pl("R2: ");
		R2.print(15, 5);
		pl("t1: ");
		t1.print(15, 5);
		pl("t2: ");
		t2.print(15, 5);

		return decomp;
	}

	public static Mat frameToMat(Frame frame) {
		Mat mat = new Mat(frame.getHeight(), frame.getWidth(), CvType.CV_8UC3);
		for (int row = 0; row < frame.getHeight(); row++) {
			for (int col = 0; col < frame.getWidth(); col++) {

				byte GREY = 0;
				byte R = 0;
				byte G = 0;
				byte B = 0;

				if (frame.getR() == null) {
					GREY = frame.getGrey()[frame.getWidth() * row + col];
					R = GREY;
					G = GREY;
					B = GREY;
				} else {

					B = frame.getB()[frame.getWidth() * row + col];
					G = frame.getG()[frame.getWidth() * row + col];
					R = frame.getR()[frame.getWidth() * row + col];
				}

				byte[] rgb = { B, G, R };
				mat.put(row, col, rgb);
			}
		}
		return mat;
	}

	public static Mat MatrixToMat(Matrix matrix) {
		Mat mat = new Mat(matrix.getRowDimension(), matrix.getColumnDimension(), CvType.CV_32F);
		for (int row = 0; row < matrix.getRowDimension(); row++) {
			for (int col = 0; col < matrix.getColumnDimension(); col++) {
				mat.put(row, col, matrix.get(row, col));
			}
		}
		return mat;
	}

	public static Matrix MatToMatrix(Mat mat) {
		Matrix matrix = new Matrix(mat.rows(), mat.cols());

		for (int row = 0; row < mat.rows(); row++) {
			for (int col = 0; col < mat.cols(); col++) {
				matrix.set(row, col, mat.get(row, col)[0]);
			}
		}

		return matrix;
	}

	public static void boxHighlight(Frame frame, MatOfKeyPoint keypoints, byte[][] RGB, int boxSize) {
		List<KeyPoint> keypointList = keypoints.toList();
		for (KeyPoint keypoint : keypointList) {
			if (true) {
				boxHighlight(frame, keypoint, RGB, boxSize * (int) Math.pow(2, keypoint.octave));
			}
		}
	}

	public static void boxHighlight(Frame frame, MatOfKeyPoint keypoints) {
		byte[][] RGB = { { 0, 0, (byte) 255 }, { (byte) 255, 0, 0 }, { 0, (byte) 255, 0 },
				{ (byte) 255, 0, (byte) 255 } };
		boxHighlight(frame, keypoints, RGB, 21);
	}

	public static void boxHighlight(Frame frame, MatOfKeyPoint keypoints, int boxSize) {
		byte[][] RGB = { { 0, 0, (byte) 255 }, { (byte) 255, 0, 0 }, { 0, (byte) 255, 0 },
				{ (byte) 255, 0, (byte) 255 } };
		boxHighlight(frame, keypoints, RGB, boxSize);
	}

	public static void boxHighlight(Frame frame, KeyPoint keypoint, byte[][] RGB, int boxSize) {

		int x = (int) keypoint.pt.x;
		int y = (int) keypoint.pt.y;
		byte[] color = getRGB(RGB, keypoint.octave);
		boxHighlight(frame, x, y, color, boxSize);

	}

	public static void boxHighlight(Frame frame, int x, int y, byte[] RGB, int boxSize) {

		int thickness = 1;
		int offset = boxSize / 2;
		byte[] color = RGB;

		// for every pixel in the box area around the point
		for (int row = 0; row < boxSize; row++) {
			for (int col = 0; col < boxSize; col++) {
				int pixelPosX = col + x - offset;
				int pixelPosY = row + y - offset;

				// do we color it?
				// check for boundaries of image
				if (pixelPosX > 0 && pixelPosX < frame.getWidth() && pixelPosY > 0 && pixelPosY < frame.getHeight()) {

					// check for distance from barrier
					if (row < thickness || row >= boxSize - thickness || col < thickness
							|| col >= boxSize - thickness) {

						// paint the pixel
						frame.getGrey()[frame.getWidth() * pixelPosY
								+ pixelPosX] = (byte) ((color[0] + color[1] + color[2]) / 3);

						if (frame.getR() != null) {
							frame.getR()[frame.getWidth() * pixelPosY + pixelPosX] = color[0];
							frame.getG()[frame.getWidth() * pixelPosY + pixelPosX] = color[1];
							frame.getB()[frame.getWidth() * pixelPosY + pixelPosX] = color[2];
						}

					}

				}
			}
		}

	}

	public static byte[] getRGB(byte[][] RGB, int octave) {
		if (octave >= RGB.length) {
			return RGB[RGB.length - 1];
		} else {
			return RGB[octave];
		}
	}

	public static void nonMaximumSuppression(MatOfKeyPoint keypoints, double patchSize) {
		List<KeyPoint> keypointList = new ArrayList<KeyPoint>(keypoints.toList());
		for (int i = 0; i < keypointList.size(); i++) {
			// find all keypoints sufficiently overlapping this one
			List<KeyPoint> interestPoints = new ArrayList<KeyPoint>();
			interestPoints.add(keypointList.get(i));

			for (int j = i + 1; j < keypointList.size(); j++) {
				if (keypointList.get(i).octave == keypointList.get(j).octave && getDistance(keypointList.get(j).pt,
						keypointList.get(i).pt) < patchSize * Math.pow(2, keypointList.get(j).octave)) {
					interestPoints.add(keypointList.get(j));
				}
			}

			if (interestPoints.size() > 1) {
				// remove all interest points
				for (int k = 1; k < interestPoints.size(); k++) {
					keypointList.remove(interestPoints.get(k));
				}

				// insert centered point
				keypointList.set(i, getStrongest(interestPoints));
			}

		}
		keypoints.fromList(keypointList);
	}

	public static double getDistance(Point a, Point b) {
		double x = a.x - b.x;
		double y = a.y - b.y;
		return Math.sqrt(x * x + y * y);
	}

	public static KeyPoint getCentroid(List<KeyPoint> keypoints) {
		float sumX = 0;
		float sumY = 0;
		for (KeyPoint keypoint : keypoints) {
			sumX += keypoint.pt.x;
			sumY += keypoint.pt.y;
		}
		sumX = sumX / keypoints.size();
		sumY = sumY / keypoints.size();
		KeyPoint centered = new KeyPoint(sumX, sumY, keypoints.get(0).size, keypoints.get(0).angle,
				keypoints.get(0).response, keypoints.get(0).octave, keypoints.get(0).class_id);
		return centered;
	}

	public static KeyPoint getStrongest(List<KeyPoint> keypoints) {
		int strongestIndex = 0;
		for (int i = 0; i < keypoints.size(); i++) {
			if (keypoints.get(strongestIndex).response < keypoints.get(i).response) {
				strongestIndex = i;
			}
		}
		return keypoints.get(strongestIndex);
	}

	public static void pruneOctaves(MatOfKeyPoint keypoints, List<Integer> octavesToKeep) {
		List<KeyPoint> keypointList = new ArrayList<KeyPoint>(keypoints.toList());
		for (int i = 0; i < keypointList.size(); i++) {
			if (!octavesToKeep.contains(keypointList.get(i).octave)) {
				keypointList.remove(i);
				i--;
			}
		}
		keypoints.fromList(keypointList);
	}

	public static Double min(ArrayList<Double> list) {
		Double min = list.get(0);
		for (int i = 1; i < list.size(); i++) {
			if (min > list.get(i)) {
				min = list.get(i);
			}
		}
		return min;
	}

	public static Double max(ArrayList<Double> list) {
		Double max = list.get(0);
		for (int i = 1; i < list.size(); i++) {
			if (max < list.get(i)) {
				max = list.get(i);
			}
		}
		return max;
	}

	public static Double sum(ArrayList<Double> list) {
		Double sum = 0.0;
		for (int i = 0; i < list.size(); i++) {
			sum += list.get(i);
		}
		return sum;
	}

	public static Double mean(ArrayList<Double> list) {
		return sum(list) / list.size();
	}

	public static Double variance(ArrayList<Double> list) {
		Double avg = mean(list);
		Double variance = 0.0;
		for (int i = 0; i < list.size(); i++) {
			variance += Math.pow(list.get(i) - avg, 2);
		}
		return variance / avg;
	}

	public static Double stdDev(ArrayList<Double> list) {
		return Math.sqrt(variance(list));
	}

	public static Double descriptorDistance(Mat desc1, Mat desc2) {
		Double sum = 0.0;
		for (int i = 0; i < desc1.cols(); i++) {
			sum += Math.pow(desc1.get(0, i)[0] - desc2.get(0, i)[0], 2);
		}
		return Math.sqrt(sum);
	}

	public static void p(Object s) {
		System.out.print(s);
	}

	public static void pl(Object s) {
		System.out.println(s);
	}
}

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

	// observations is a list of size n (points) with inner lists of size m
	// (cameras). If 3 cameras and 10 points, observations is a list of size 10,
	// with the sublists being size 3
	public static void bundleAdjust(ArrayList<Pose> cameras, ArrayList<Point3D> point3Ds,
			ArrayList<ArrayList<Point2D>> observations, int numIterations) {

		double fx = CameraIntrinsics.getFx();
		double fy = CameraIntrinsics.getFy();
		double s = CameraIntrinsics.getS();
		double cx = CameraIntrinsics.getCx();
		double cy = CameraIntrinsics.getCy();

		for (int i = 0; i < numIterations; i++) {

			Matrix J = new Matrix(2 * point3Ds.size() * cameras.size(), 3 * point3Ds.size() + 12 * cameras.size());
			Matrix r = new Matrix(2 * point3Ds.size() * cameras.size(), 1);

			// calculate J and r
			for (int p = 0; p < point3Ds.size(); p++) {
				for (int c = 0; c < cameras.size(); c++) {

					// establish variables
					double x = point3Ds.get(p).getX();
					double y = point3Ds.get(p).getY();
					double z = point3Ds.get(p).getZ();
					double tx = cameras.get(c).getTx();
					double ty = cameras.get(c).getTy();
					double tz = cameras.get(c).getTz();
					double r00 = cameras.get(c).getR00();
					double r01 = cameras.get(c).getR01();
					double r02 = cameras.get(c).getR02();
					double r10 = cameras.get(c).getR10();
					double r11 = cameras.get(c).getR11();
					double r12 = cameras.get(c).getR12();
					double r20 = cameras.get(c).getR20();
					double r21 = cameras.get(c).getR21();
					double r22 = cameras.get(c).getR22();

					// establish a posteriori
					double w = x * r20 + y * r21 + z * r22 + tz;
					if (w == 0) {
						w = 1;
					}
					double wSquared = w * w;
					double fuTop = x * (fx * r00 + s * r10 + cx * r20) + y * (fx * r01 + s * r11 + cx * r21)
							+ z * (fx * r02 + s * r12 + cx * r22) + fx * tx + s * ty + cx * tz;
					double fvTop = x * (fy * r10 + cy * r20) + y * (fy * r11 + cy * r21) + z * (fy * r12 + cy * r22)
							+ fy * ty + cy * tz;

					double fu = fuTop / w;
					double fv = fvTop / w;

					// if the camera did not see this point, skip these rows
					if (observations.get(p).get(c) == null) {
						continue;
					}

					// calculate partial derivatives
					double dfudx = ((fx * r00 + s * r10 + cx * r20) * w - fuTop * r20) / wSquared;
					double dfvdx = ((fy * r10 + cy * r20) * w - fvTop * r20) / wSquared;
					double dfudy = ((fx * r01 + s * r11 + cx * r21) * w - fuTop * r21) / wSquared;
					double dfvdy = ((fy * r11 + cy * r21) * w - fvTop * r21) / wSquared;
					double dfudz = ((fx * r02 + s * r12 + cx * r22) * w - fuTop * r22) / wSquared;
					double dfvdz = ((fy * r12 + cy * r22) * w - fvTop * r22) / wSquared;
					double dfudtx = fx / w;
					double dfvdtx = 0;
					double dfudty = s / w;
					double dfvdty = fy / w;
					double dfudtz = (cx * w - fuTop) / wSquared;
					double dfvdtz = (cy * w - fvTop) / wSquared;
					double dfudr00 = x * fx / w;
					double dfvdr00 = 0;
					double dfudr01 = y * fx / w;
					double dfvdr01 = 0;
					double dfudr02 = z * fx / w;
					double dfvdr02 = 0;
					double dfudr10 = x * s / w;
					double dfvdr10 = x * fy / w;
					double dfudr11 = y * s / w;
					double dfvdr11 = y * fy / w;
					double dfudr12 = z * s / w;
					double dfvdr12 = z * fy / w;
					double dfudr20 = (x * cx * w - fuTop * x) / wSquared;
					double dfvdr20 = (x * cy * w - fvTop * x) / wSquared;
					double dfudr21 = (y * cx * w - fuTop * y) / wSquared;
					double dfvdr21 = (y * cy * w - fvTop * y) / wSquared;
					double dfudr22 = (z * cx * w - fuTop * z) / wSquared;
					double dfvdr22 = (z * cy * w - fvTop * z) / wSquared;

					// set in J
					int rowU = 2 * cameras.size() * p + 2 * c;
					int rowV = rowU + 1;

					// u constraint
					J.set(rowU, 12 * c + 0, dfudr00);
					J.set(rowU, 12 * c + 1, dfudr01);
					J.set(rowU, 12 * c + 2, dfudr02);
					J.set(rowU, 12 * c + 3, dfudr10);
					J.set(rowU, 12 * c + 4, dfudr11);
					J.set(rowU, 12 * c + 5, dfudr12);
					J.set(rowU, 12 * c + 6, dfudr20);
					J.set(rowU, 12 * c + 7, dfudr21);
					J.set(rowU, 12 * c + 8, dfudr22);
					J.set(rowU, 12 * c + 9, dfudtx);
					J.set(rowU, 12 * c + 10, dfudty);
					J.set(rowU, 12 * c + 11, dfudtz);

					J.set(rowU, 12 * cameras.size() + 3 * p + 0, dfudx);
					J.set(rowU, 12 * cameras.size() + 3 * p + 1, dfudy);
					J.set(rowU, 12 * cameras.size() + 3 * p + 2, dfudz);

					// v constraint
					J.set(rowV, 12 * c + 0, dfvdr00);
					J.set(rowV, 12 * c + 1, dfvdr01);
					J.set(rowV, 12 * c + 2, dfvdr02);
					J.set(rowV, 12 * c + 3, dfvdr10);
					J.set(rowV, 12 * c + 4, dfvdr11);
					J.set(rowV, 12 * c + 5, dfvdr12);
					J.set(rowV, 12 * c + 6, dfvdr20);
					J.set(rowV, 12 * c + 7, dfvdr21);
					J.set(rowV, 12 * c + 8, dfvdr22);
					J.set(rowV, 12 * c + 9, dfvdtx);
					J.set(rowV, 12 * c + 10, dfvdty);
					J.set(rowV, 12 * c + 11, dfvdtz);

					J.set(rowV, 12 * cameras.size() + 3 * p + 0, dfvdx);
					J.set(rowV, 12 * cameras.size() + 3 * p + 1, dfvdy);
					J.set(rowV, 12 * cameras.size() + 3 * p + 2, dfvdz);

					// set r variables
					r.set(rowU, 0, Math.pow(observations.get(p).get(c).getX() - fu, 2));
					r.set(rowV, 0, Math.pow(observations.get(p).get(c).getY() - fv, 2));

				}
			}

			pl("J:");
			J.print(10, 3);
			pl("r:");
			r.print(15, 5);

			Matrix JtJ = J.transpose().times(J);
			pl("JtJ:");
			JtJ.print(15, 3);

			Matrix U = JtJ.getMatrix(0, 12 * cameras.size() - 1, 0, 12 * cameras.size() - 1);
			Matrix V = JtJ.getMatrix(12 * cameras.size(), JtJ.getRowDimension() - 1, 12 * cameras.size(),
					JtJ.getRowDimension() - 1);
			Matrix W = JtJ.getMatrix(0, 12 * cameras.size() - 1, 12 * cameras.size(), JtJ.getColumnDimension() - 1);

			Matrix deltaC = JtJInv.times(J.transpose()).times(r);
			pl("deltaC: ");
			deltaC.print(15, 5);

			// update the points
			for (int p = 0; p < point3Ds.size(); p++) {
				point3Ds.get(p).setX(point3Ds.get(p).getX() + deltaC.get(12 * cameras.size() + 3 * p + 0, 0));
				point3Ds.get(p).setY(point3Ds.get(p).getY() + deltaC.get(12 * cameras.size() + 3 * p + 1, 0));
				point3Ds.get(p).setZ(point3Ds.get(p).getZ() + deltaC.get(12 * cameras.size() + 3 * p + 2, 0));
			}

			// update the poses
			for (int c = 0; c < cameras.size(); c++) {
				cameras.get(c).setR00(cameras.get(c).getR00() + deltaC.get(12 * c + 0, 0));
				cameras.get(c).setR01(cameras.get(c).getR01() + deltaC.get(12 * c + 1, 0));
				cameras.get(c).setR02(cameras.get(c).getR02() + deltaC.get(12 * c + 2, 0));
				cameras.get(c).setR10(cameras.get(c).getR10() + deltaC.get(12 * c + 3, 0));
				cameras.get(c).setR11(cameras.get(c).getR11() + deltaC.get(12 * c + 4, 0));
				cameras.get(c).setR12(cameras.get(c).getR12() + deltaC.get(12 * c + 5, 0));
				cameras.get(c).setR20(cameras.get(c).getR20() + deltaC.get(12 * c + 6, 0));
				cameras.get(c).setR21(cameras.get(c).getR21() + deltaC.get(12 * c + 7, 0));
				cameras.get(c).setR22(cameras.get(c).getR22() + deltaC.get(12 * c + 8, 0));
				cameras.get(c).setTx(cameras.get(c).getTx() + deltaC.get(12 * c + 9, 0));
				cameras.get(c).setTy(cameras.get(c).getTy() + deltaC.get(12 * c + 10, 0));
				cameras.get(c).setTz(cameras.get(c).getTz() + deltaC.get(12 * c + 11, 0));
			}
		}

	}

	public static Matrix quaternionToRotationMatrix(Matrix Q) {
		double q1 = Q.get(0, 0);
		double q2 = Q.get(1, 0);
		double q3 = Q.get(2, 0);
		double q4 = Q.get(3, 0);
		Matrix R = Matrix.identity(4, 4);

		// R.set(0, 0, q1 * q1 + q2 * q2 - q3 * q3 - q4 * q4);
		// R.set(0, 1, 2 * (q2 * q3 - q1 * q4));
		// R.set(0, 2, 2 * (q2 * q4 + q1 * q3));
		// R.set(1, 0, 2 * (q2 * q3 + q1 * q4));
		// R.set(1, 1, q1 * q1 - q2 * q2 + q3 * q3 - q4 * q4);
		// R.set(1, 2, 2 * (q2 * q4 - q1 * q2));
		// R.set(2, 0, 2 * (q2 * q4 - q1 * q3));
		// R.set(2, 1, 2 * (q3 * q4 + q1 * q2));
		// R.set(2, 2, q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4);

		double qw = Q.get(0, 0);
		double qx = Q.get(1, 0);
		double qy = Q.get(2, 0);
		double qz = Q.get(3, 0);

		R.set(0, 0, 1 - 2 * qy * qy - 2 * qz * qz);
		R.set(0, 1, 2 * qx * qy - 2 * qz * qw);
		R.set(0, 2, 2 * qx * qz + 2 * qy * qw);
		R.set(1, 0, 2 * qx * qy + 2 * qz * qw);
		R.set(1, 1, 1 - 2 * qx * qx - 2 * qz * qz);
		R.set(1, 2, 2 * qy * qz - 2 * qx * qw);
		R.set(2, 0, 2 * qx * qz - 2 * qy * qw);
		R.set(2, 1, 2 * qy * qz + 2 * qx * qw);
		R.set(2, 2, 1 - 2 * qx * qx - 2 * qy * qy);

		return R;
	}

	public static Matrix getQuaternion(Matrix R) {
		// index starting with 1
		double r11 = R.get(0, 0);
		double r12 = R.get(0, 1);
		double r13 = R.get(0, 2);
		double r21 = R.get(1, 0);
		double r22 = R.get(1, 1);
		double r23 = R.get(1, 2);
		double r31 = R.get(2, 0);
		double r32 = R.get(2, 1);
		double r33 = R.get(2, 2);

		// using Sarabandi-Thomas method
		double n = -1;
		double q1 = 0;
		double q2 = 0;
		double q3 = 0;
		double q4 = 0;

		// q1
		if (r11 + r22 + r33 > n) {
			q1 = 0.5 * Math.sqrt(1 + r11 + r22 + r33);
		} else {
			q1 = 0.5 * Math.sqrt(((r32 - r23) * (r32 - r23) + (r13 - r31) * (r13 - r31) + (r21 - r12) * (r21 - r12))
					/ (3 - r11 - r22 - r33));
		}

		// q2
		if (r11 - r22 - r33 > n) {
			q2 = 0.5 * Math.sqrt(1 + r11 - r22 - r33);
		} else {
			q2 = 0.5 * Math.sqrt(((r32 - r23) * (r32 - r23) + (r12 + r21) * (r12 + r21) + (r31 + r13) * (r31 + r13))
					/ (3 - r11 + r22 + r33));
		}

		// q3
		if (-r11 + r22 - r33 > n) {
			q3 = 0.5 * Math.sqrt(1 - r11 + r22 - r33);
		} else {
			q3 = 0.5 * Math.sqrt(((r13 - r31) * (r13 - r31) + (r12 + r21) * (r12 + r21) + (r23 + r32) * (r23 + r32))
					/ (3 + r11 - r22 + r33));
		}

		// q4
		if (-r11 - r22 + r33 > n) {
			q4 = 0.5 * Math.sqrt(1 - r11 - r22 + r33);
		} else {
			q4 = 0.5 * Math.sqrt(((r21 - r12) * (r21 - r12) + (r31 + r13) * (r31 + r13) + (r32 + r23) * (r32 + r23))
					/ (3 + r11 + r22 - r33));
		}

		// assign signs
		q2 = r32 - r23 < 0 ? -q2 : q2;
		q3 = r13 - r31 < 0 ? -q3 : q3;
		q4 = r21 - r12 < 0 ? -q4 : q4;

		Matrix Q = new Matrix(4, 1);
		Q.set(0, 0, q1);
		Q.set(1, 0, q2);
		Q.set(2, 0, q3);
		Q.set(3, 0, q4);

		return Q;

	}

	public static Matrix setQuatFromMatrix(Matrix mat) {
		double az, ay, ax;
		double ai, aj, ak;
		double si, sj, sk;
		double ci, cj, ck;
		double cy, cc, cs, sc, ss;

		cy = Math.sqrt(mat.get(0, 0) * mat.get(0, 0) + mat.get(1, 0) * mat.get(1, 0));

		if (cy > 0.00000001) {
			ax = Math.atan2(mat.get(2, 1), mat.get(2, 2));
			ay = Math.atan2(-mat.get(2, 0), cy);
			az = Math.atan2(mat.get(1, 0), mat.get(0, 0));
		} else {
			ax = Math.atan2(-mat.get(1, 2), mat.get(1, 1));
			ay = Math.atan2(-mat.get(2, 0), cy);
			az = 0.0;
		}

		ai = ax / 2.0;
		aj = ay / 2.0;
		ak = az / 2.0;

		ci = Math.cos(ai);
		si = Math.sin(ai);
		cj = Math.cos(aj);
		sj = Math.sin(aj);
		ck = Math.cos(ak);
		sk = Math.sin(ak);
		cc = ci * ck;
		cs = ci * sk;
		sc = si * ck;
		ss = si * sk;

		double qx = cj * sc - sj * cs;
		double qy = cj * ss + sj * cc;
		double qz = cj * cs - sj * sc;
		double qw = cj * cc + sj * ss;

		Matrix Q = new Matrix(4, 1);
		Q.set(0, 0, qw);
		Q.set(1, 0, qx);
		Q.set(2, 0, qy);
		Q.set(3, 0, qz);

		return Q;
	}

	public static Matrix PnP(ArrayList<Point3D> points3D, ArrayList<Point2D> points2D) {
		Matrix B = new Matrix(2 * points3D.size(), 12);

		for (int i = 0; i < points3D.size(); i++) {
			Point3D point3D = points3D.get(i);
			Point2D point2D = points2D.get(i);

			B.set(2 * i, 0, point3D.getX());
			B.set(2 * i, 1, point3D.getY());
			B.set(2 * i, 2, point3D.getZ());
			B.set(2 * i, 3, 1);
			B.set(2 * i, 8, -point2D.getX() * point3D.getX());
			B.set(2 * i, 9, -point2D.getX() * point3D.getY());
			B.set(2 * i, 10, -point2D.getX() * point3D.getZ());
			B.set(2 * i, 11, -point2D.getX());

			B.set(2 * i + 1, 4, point3D.getX());
			B.set(2 * i + 1, 5, point3D.getY());
			B.set(2 * i + 1, 6, point3D.getZ());
			B.set(2 * i + 1, 7, 1);
			B.set(2 * i + 1, 8, -point2D.getY() * point3D.getX());
			B.set(2 * i + 1, 9, -point2D.getY() * point3D.getY());
			B.set(2 * i + 1, 10, -point2D.getY() * point3D.getZ());
			B.set(2 * i + 1, 11, -point2D.getY());
		}
		SingularValueDecomposition svd = B.svd();
		// pl("V: ");
		// svd.getV().print(15, 5);
		// pl("B: ");
		// B.print(15, 5);
		Matrix p = svd.getV().getMatrix(0, 11, 11, 11);

		// pl("p: ");
		// p.print(15, 5);
		Matrix pi = new Matrix(3, 4);

		pi.set(0, 0, p.get(0, 0));
		pi.set(0, 1, p.get(1, 0));
		pi.set(0, 2, p.get(2, 0));
		pi.set(0, 3, p.get(3, 0));
		pi.set(1, 0, p.get(4, 0));
		pi.set(1, 1, p.get(5, 0));
		pi.set(1, 2, p.get(6, 0));
		pi.set(1, 3, p.get(7, 0));
		pi.set(2, 0, p.get(8, 0));
		pi.set(2, 1, p.get(9, 0));
		pi.set(2, 2, p.get(10, 0));
		pi.set(2, 3, p.get(11, 0));

		// pl("pi: ");
		// pi.print(15, 5);

		pi = pi.times(1 / pi.getMatrix(2, 2, 0, 2).normF());
		Matrix X = new Matrix(4, 1);
		X.set(0, 0, points3D.get(0).getX());
		X.set(1, 0, points3D.get(0).getY());
		X.set(2, 0, points3D.get(0).getZ());
		X.set(3, 0, 1);
		Matrix x = pi.times(X);
		// pl("x: ");
		// x.print(15, 5);

		if (x.get(2, 0) < 0) {
			pi = pi.times(-1);
		}

		// pl("pi normalized: ");
		// pi.print(15, 5);

		Matrix E = CameraIntrinsics.getK().inverse().times(pi);
		pl("E: ");
		E.print(15, 5);

		return E;
	}

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

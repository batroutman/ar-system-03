package ARPipeline;

import org.opencv.core.Mat;

public class ActiveSearchData {

	public static int[] PATCH_SIZES = { 10, 15, 20, 25, 30 };
	public static int LARGE_BOX = PATCH_SIZES[4];

	protected int patchLevel = 1;

	protected Point2D lastLocation = null;
	protected Mat lastDescriptor = null;
	protected Long lastFrameObserved = null;
	protected Double dx = null;
	protected Double dy = null;

	// dx and dy averaged across last n frames (makes active search robust to
	// duplicate frames)
	protected int n = 20;
	protected Double dxN = null;
	protected Double dyN = null;

	public Point2D getLastLocation() {
		return lastLocation;
	}

	public void setLastLocation(Point2D lastLocation) {
		this.lastLocation = lastLocation;
	}

	public Mat getLastDescriptor() {
		return lastDescriptor;
	}

	public void setLastDescriptor(Mat lastDescriptor) {
		this.lastDescriptor = lastDescriptor;
	}

	public Long getLastFrameObserved() {
		return lastFrameObserved;
	}

	public void setLastFrameObserved(Long lastFrameObserved) {
		this.lastFrameObserved = lastFrameObserved;
	}

	public Double getDx() {
		return dx;
	}

	public void setDx(Double dx) {
		this.dx = dx;
	}

	public Double getDy() {
		return dy;
	}

	public void setDy(Double dy) {
		this.dy = dy;
	}

	public int getPatchLevel() {
		return patchLevel;
	}

	public void setPatchLevel(int patchLevel) {
		if (patchLevel < PATCH_SIZES.length)
			this.patchLevel = patchLevel;
	}

	public void increaseWindowSize() {
		this.patchLevel = this.patchLevel >= PATCH_SIZES.length - 1 ? this.patchLevel : this.patchLevel + 1;
	}

	public void decreaseWindowSize() {
		this.patchLevel = this.patchLevel == 0 ? this.patchLevel : this.patchLevel - 1;
	}

	public int getWindowSize() {
		return PATCH_SIZES[this.patchLevel];
	}

	public int getN() {
		return n;
	}

	public void setN(int n) {
		this.n = n;
	}

	public Double getDxN() {
		return dxN;
	}

	public void setDxN(Double dxN) {
		this.dxN = dxN;
	}

	public Double getDyN() {
		return dyN;
	}

	public void setDyN(Double dyN) {
		this.dyN = dyN;
	}

	public void registerDx(double dx) {
		double oldDxN = this.dxN;
		this.dxN = (this.dxN * this.n - oldDxN + dx) / n;
		this.dx = dx;
	}

	public void registerDy(double dy) {
		double oldDyN = this.dyN;
		this.dyN = (this.dyN * this.n - oldDyN + dy) / n;
		this.dy = dy;
	}

}

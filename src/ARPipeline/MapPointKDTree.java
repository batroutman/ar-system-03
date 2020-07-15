package ARPipeline;

import java.util.ArrayList;

import org.opencv.core.Mat;

public class MapPointKDTree {

	protected MapPointKDTree parent = null;
	protected MapPointKDTree left = null;
	protected MapPointKDTree right = null;
	protected int index = 0;
	protected MapPoint payload = null;
	
	protected MapPointKDTree() {
		
	}
	
	public MapPointKDTree(MapPoint mp) {
		this.index = 0;
		this.payload = mp;
	}
	
	public MapPointKDTree(MapPointKDTree parent, MapPoint mp, int index) {
		this.index = index % mp.descriptor.cols();
		this.parent = parent;
		this.payload = mp;
	}
	
	public void insert(MapPoint mp) {
		double elementVal = mp.descriptor.get(0, this.index)[0];
		double payloadVal = this.payload.descriptor.get(0, this.index)[0];
		if (elementVal < payloadVal) {
			if (this.left != null) {
				this.left.insert(mp);
			} else {
				this.left = new MapPointKDTree(this, mp, this.index + 1);
			}
		} else {
			if (this.right != null) {
				this.right.insert(mp);
			} else {
				this.right = new MapPointKDTree(this, mp, this.index + 1);
			}
		}
	}

	
	public static Double scalarDistance(Double pt1, Double pt2) {
		return Math.abs(pt1 - pt2);
	}
	
	// return the nearest neighbor to the given map point
		public MapPoint nearest(Mat descriptor) {
			return this.nearest(descriptor, this.payload, ARUtils.descriptorDistance(this.payload.descriptor, descriptor));
		}
	
	// return the nearest neighbor to the given map point
	public MapPoint nearest(MapPoint mp) {
		return this.nearest(mp.descriptor, this.payload, ARUtils.descriptorDistance(this.payload.descriptor, mp.descriptor));
	}
	
	protected MapPoint nearest(Mat descriptor, MapPoint best, Double bestDist) {
		
		// check if this is a closer node
		MapPoint nextBest;
		Double nextBestDist;
		Double payloadToMpDist = ARUtils.descriptorDistance(this.payload.descriptor, descriptor);
		if (payloadToMpDist < bestDist) {
			nextBest = this.payload;
			nextBestDist = payloadToMpDist;
		} else {
			nextBest = best;
			nextBestDist = bestDist;
		}
		
		// find best node between the current best and the ideal subtree
		double elementVal = descriptor.get(0, this.index)[0]; // hyperplane value
		double payloadVal = this.payload.descriptor.get(0, this.index)[0];
		if (elementVal < payloadVal) {
			if (this.left != null) {
				nextBest = this.left.nearest(descriptor, nextBest, nextBestDist);
			}
		} else {
			if (this.right != null) {
				nextBest = this.right.nearest(descriptor, nextBest, nextBestDist);
			}
		}
		nextBestDist = ARUtils.descriptorDistance(descriptor, nextBest.descriptor);
		
		// check the in-ideal subtree
		// if dist from nextBest to hyperplane is less than dist from nextBest to the target point, we cannot rule out the other subtree. Check it.
		if (scalarDistance(elementVal, nextBest.descriptor.get(0, this.index)[0]) < nextBestDist) {
			if (elementVal < payloadVal) {
				if (this.right != null) {
					nextBest = this.right.nearest(descriptor, nextBest, nextBestDist);
				}
			} else {
				if (this.left != null) {
					nextBest = this.left.nearest(descriptor, nextBest, nextBestDist);
				}
			}
		}
		
		return nextBest;
		
	}
	
	
	public int depth() {
		if (this.left == null && this.right == null) {
			return 1;
		} else {
			int leftDepth = 0;
			int rightDepth = 0;
			if (this.left != null) {
				leftDepth = this.left.depth();
			}
			if (this.right != null) {
				rightDepth = this.right.depth();
			}
			if (leftDepth > rightDepth) {
				return leftDepth + 1;
			} else {
				return rightDepth + 1;
			}
		}
	}
	
	
	
	
}

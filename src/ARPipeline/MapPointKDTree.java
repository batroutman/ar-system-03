package ARPipeline;

import java.util.ArrayList;

import org.opencv.core.Mat;

public class MapPointKDTree {

	protected MapPointKDTree parent = null;
	protected MapPointKDTree left = null;
	protected MapPointKDTree right = null;
	protected int index = 0;
	protected double splitVal = 128;
	protected MapPoint payload = null;
	
	protected MapPointKDTree() {
		
	}
	
	public MapPointKDTree(MapPoint mp) {
		this.index = 0;
		this.payload = mp;
	}
	
	public MapPointKDTree(MapPointKDTree parent, MapPoint mp, int index) {
		this.index = index % mp.descriptor.rows();
		this.parent = parent;
		this.payload = mp;
	}
	
	public void insert(MapPoint mp) {
		double elementVal = mp.descriptor.get(0, this.index)[0];
		if (elementVal < this.splitVal) {
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


	
	
	public static Double descriptorDistance(Mat desc1, Mat desc2) {
		Double sum = 0.0;
		for (int i = 0; i < desc1.cols(); i++) {
			sum += Math.pow(desc1.get(0, i)[0] - desc2.get(0,  i)[0], 2);
		}
		return Math.sqrt(sum);
	}
	
	
	
	
}

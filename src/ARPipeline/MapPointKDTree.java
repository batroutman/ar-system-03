package ARPipeline;

import java.util.ArrayList;

public class MapPointKDTree {

	protected MapPointKDTree parent = null;
	protected MapPointKDTree left = null;
	protected MapPointKDTree right = null;
	protected int index = 0;
	protected int depth = 0;
	protected double splitVal = 128;
	protected ArrayList<MapPoint> payload = null;
	
	protected MapPointKDTree() {
		
	}
	
	public MapPointKDTree(int depth) {
		this.index = 0;
		this.depth = depth;
		this.parent = null;
		if (depth > 0) {
			this.left = new MapPointKDTree(this, depth - 1, index + 1);
			this.right = new MapPointKDTree(this, depth - 1, index + 1);
		} else {
			this.payload = new ArrayList<MapPoint>();
		}
	}
	
	protected MapPointKDTree(MapPointKDTree parent, int depth, int index) {
		this.index = index;
		this.depth = depth;
		this.parent = parent;
		if (depth > 0) {
			this.left = new MapPointKDTree(this, depth - 1, index + 1);
			this.right = new MapPointKDTree(this, depth - 1, index + 1);
		} else {
			this.payload = new ArrayList<MapPoint>();
		}
	}
	
	public void insert(MapPoint mp) {
		if (this.depth > 0) {
			double elementValue = mp.descriptor.get(0, this.index)[0];
			if (elementValue < this.splitVal) {
				this.left.insert(mp);
			} else {
				this.right.insert(mp);
			}
		} else {
			this.payload.add(mp);
		}
	}
	
	
}

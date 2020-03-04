
#ifndef KDTREE_H_
#define KDTREE_H_

#include <memory>


// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	std::unique_ptr<Node> left;
	std::unique_ptr<Node> right;

	Node(const PointT &pnt, int setId)
	:	point(pnt), id(setId), left(nullptr), right(nullptr)
	{}

};


template<typename PointT>
struct KdTree
{
	std::unique_ptr<Node<PointT>> root;

	KdTree()
	: root(nullptr)	{}

	void insert(const PointT &point, int id)
	{
		// the function should create a new node and place correctly with in the root
		if (root == nullptr)
		{
			root = std::unique_ptr<Node<PointT>>(new Node<PointT>(point, id));
		}
		else
		{
			// traverse tree
			const int pointDimensions = 3;
			int depth = 0;
			// alias current node
			Node<PointT> *node = root.get();
			// TODO: restrict depth to a max value
			while (true) {
				const int check_index = depth % pointDimensions;
				if (point.data[check_index] < node->point.data[check_index]) {
					if (node->left == nullptr) {
						node->left = std::unique_ptr<Node<PointT>>(new Node<PointT>(point, id));
						break;
					} else {
						node = node->left.get();
					}
				} else {
					if (node->right == nullptr) {
						node->right = std::unique_ptr<Node<PointT>>(new Node<PointT>(point, id));
						break;
					} else {
						node = node->right.get();
					}
				}
				depth++;
			}
		}

	}

	void searchHelper(const PointT &target, const float distanceTol, std::unique_ptr<Node<PointT>> &node, const int depth, std::vector<int> &ids) {
		const int pointDimensions = 3;
		if (node != nullptr) {
			const int dim = depth % pointDimensions;
			if (target.data[dim] + distanceTol < node->point.data[dim]) {
				// not in box, recurse into left only
				searchHelper(target, distanceTol, node->left, depth + 1, ids);
			} else if (target.data[dim] - distanceTol > node->point.data[dim]) {
				// not in box, recurse into right only
				searchHelper(target, distanceTol, node->right, depth + 1, ids);
			} else {
				// target is in this dimension's box range
				// check remaining dimensions and radius
				bool inside_box = true;
				double sumsqr = std::pow(target.data[dim] - node->point.data[dim], 2.0);
				for (int k = 1; k < pointDimensions; ++k) {
					const int test_dim = (dim + k) % pointDimensions;
					if ((target.data[test_dim] + distanceTol < node->point.data[test_dim]) ||
					    (target.data[test_dim] - distanceTol > node->point.data[test_dim])) {
						inside_box = false;
						break;
					} else {
						sumsqr += std::pow(target.data[test_dim] - node->point.data[test_dim], 2.0);
					}
				}
				if (inside_box && std::sqrt(sumsqr) <= distanceTol) {
					ids.push_back(node->id);
				}
				// recurse into both sides
				searchHelper(target, distanceTol, node->left, depth + 1, ids);
				searchHelper(target, distanceTol, node->right, depth + 1, ids);
			}

		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const PointT &target, float distanceTol)
	{
		std::vector<int> ids;
		if (root == nullptr)
		{
			PCL_ERROR ("KdTree has no points \n");
		}
		else
		{
			// traverse tree
			searchHelper(target, distanceTol, root, 0, ids);
		}

		return ids;
	}

};

#endif // KDTREE_H_

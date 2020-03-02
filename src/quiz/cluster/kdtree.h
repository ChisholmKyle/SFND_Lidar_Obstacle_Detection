/* \author Aaron Brown */
// Quiz on implementing kd tree

#include <memory>
#include "render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	std::unique_ptr<Node> left;
	std::unique_ptr<Node> right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(nullptr), right(nullptr)
	{}

};

struct KdTree
{
	std::unique_ptr<Node> root;

	KdTree()
	: root(nullptr)	{}

	void insert(std::vector<float> point, int id)
	{
		// the function should create a new node and place correctly with in the root
		if (root == nullptr)
		{
			root = std::unique_ptr<Node>(new Node(point, id));
		}
		else
		{
			// check dimension of point
			if (point.size() != root->point.size()) {
				PCL_ERROR ("Wrong point size for KdTree \n");
			}

			// traverse tree
			const int point_dimensions = point.size();
			int depth = 0;
			// alias current node
			Node *node = root.get();
			// TODO: restrict depth to a max value
			while (true) {
				const int check_index = depth % point_dimensions;
				if (point[check_index] < node->point[check_index]) {
					if (node->left == nullptr) {
						node->left = std::unique_ptr<Node>(new Node(point, id));
						break;
					} else {
						node = node->left.get();
					}
				} else {
					if (node->right == nullptr) {
						node->right = std::unique_ptr<Node>(new Node(point, id));
						break;
					} else {
						node = node->right.get();
					}
				}
				depth++;
			}
		}

	}

	void searchHelper(const std::vector<float> &target, const float distanceTol, std::unique_ptr<Node> &node, const int depth, std::vector<int> &ids) {
		if (node != nullptr) {
			const int dim = depth % target.size();
			if (target[dim] + distanceTol < node->point[dim]) {
				// not in box, recurse into left only
				searchHelper(target, distanceTol, node->left, depth + 1, ids);
			} else if (target[dim] - distanceTol > node->point[dim]) {
				// not in box, recurse into right only
				searchHelper(target, distanceTol, node->right, depth + 1, ids);
			} else {
				// target is in this dimension's box range
				// check remaining dimensions and radius
				bool inside_box = true;
				double sumsqr = std::pow(target[dim] - node->point[dim], 2.0);
				for (int k = 1; k < target.size(); ++k) {
					const int test_dim = (dim + k) % target.size();
					if ((target[test_dim] + distanceTol < node->point[test_dim]) ||
					    (target[test_dim] - distanceTol > node->point[test_dim])) {
						inside_box = false;
						break;
					} else {
						sumsqr += std::pow(target[test_dim] - node->point[test_dim], 2.0);
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
	std::vector<int> search(const std::vector<float> &target, float distanceTol)
	{
		std::vector<int> ids;
		if (root == nullptr)
		{
			PCL_ERROR ("KdTree has no points \n");
		}
		else
		{
			// check dimension of point
			if (target.size() != root->point.size()) {
				PCL_ERROR ("Wrong point size for KdTree \n");
			}
			// traverse tree
			searchHelper(target, distanceTol, root, 0, ids);
		}

		return ids;
	}

};

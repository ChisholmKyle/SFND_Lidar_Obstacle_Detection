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

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}

};





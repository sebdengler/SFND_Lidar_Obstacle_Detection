/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertRecursive(Node*& node, std::vector<float> point, int id, int splitDim)
	{
		if (node == NULL)
		{
			node = new Node(point, id);
		}
		else if (point[splitDim] < node->point[splitDim])
		{
			insertRecursive(node->left, point, id, (splitDim+1)%2);
		}
		else
		{
			insertRecursive(node->right, point, id, (splitDim+1)%2);
		}
		
	}

	void insert(std::vector<float> point, int id)
	{	
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		insertRecursive(root, point, id, 0);
	}

	float distance(std::vector<float> point1, std::vector<float> point2)
	{
		float x = point1[0] - point2[0];
		float y = point1[1] - point2[1];
		float dist = sqrt(x*x + y*y);
		return dist;
	}

	bool inBox(std::vector<float> target, std::vector<float> reference, float distanceTol)
	{
		if (target[0]-reference[0] <= distanceTol && target[1]-reference[1] <= distanceTol)
			return true;
		else
			return false;
	}

	void searchRecursive(Node* node, std::vector<float> target, float distanceTol, int dim, std::vector<int>& ids)
	{		
		if (node == NULL)
			return;

		if (inBox(target, node->point, distanceTol))
		{
			if (distance(target, node->point) < distanceTol)
				ids.push_back(node->id);
		}

		if (target[dim]-distanceTol < node->point[dim])
			searchRecursive(node->left, target, distanceTol, (dim+1)%2, ids);
		if (target[dim]+distanceTol >= node->point[dim])
			searchRecursive(node->right, target, distanceTol, (dim+1)%2, ids);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchRecursive(root, target, distanceTol, 0, ids);
		return ids;
	}
	

};





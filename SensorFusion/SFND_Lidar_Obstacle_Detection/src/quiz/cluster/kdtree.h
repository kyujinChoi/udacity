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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		int depth = 0;
		Node* item = new Node(point, id);

		Node **cur_node = &root;
		while(*cur_node != NULL)
		{
			if (depth % 2 == 0)
			{
				if (item->point[0] < (*cur_node)->point[0])
					cur_node = &(*cur_node)->left;
				else
					cur_node = &(*cur_node)->right;
			}
			else
			{
				if (item->point[1] < (*cur_node)->point[1])
					cur_node = &(*cur_node)->left;
				else
					cur_node = &(*cur_node)->right;
			}
			depth++;
		}
		*cur_node = item;
		// std::cout << (*cur_node)->point[0] << ", " << (*cur_node)->point[1] << std::endl;
		// std::cout << (*cur_node)->point[0] << ", " << (*cur_node)->point[1] << std::endl;
		// std::cout << root->point[0] << ", " << root->point[1] << std::endl;
		// if(root->left == NULL)
		// 	std::cout << "left NULL" << std::endl;
		// if(root->right == NULL)
		// 	std::cout << "right NULL" << std::endl;
	}

	// return a list of point ids in the tree that are within distance of target
	void recursiveSearch(std::vector<float> target, float distanceTol, Node **cur_node, int* depth, std::vector<int> &ids)
	{
		if(*cur_node == NULL) return;

		if(fabs((*cur_node)->point[0] - target[0]) <= distanceTol && fabs((*cur_node)->point[1] - target[1]) <= distanceTol)
		{
			float dist = sqrt(((*cur_node)->point[0] - target[0]) * ((*cur_node)->point[0] - target[0]) + ((*cur_node)->point[1] - target[1]) * ((*cur_node)->point[1] - target[1]));
			if(dist <= distanceTol)
				ids.push_back((*cur_node)->id);
		}
			
		(*depth)++;
		if ((target[*depth % 2] - distanceTol) < (*cur_node)->point[*depth % 2])
			recursiveSearch(target, distanceTol, &(*cur_node)->left, depth, ids);
		if ((target[*depth % 2] + distanceTol) > (*cur_node)->point[*depth % 2])
			recursiveSearch(target, distanceTol, &(*cur_node)->right, depth, ids);
	}
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		int depth = 0;

		Node **cur_node = &root;
		recursiveSearch(target, distanceTol, cur_node, &depth, ids);
		return ids;
	}
	

};





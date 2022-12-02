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
			std::cout << "depth : " << depth << std::endl;
		}
		*cur_node = item;
		// std::cout << (*cur_node)->point[0] << ", " << (*cur_node)->point[1] << std::endl;
		std::cout << (*cur_node)->point[0] << ", " << (*cur_node)->point[1] << std::endl;
		std::cout << root->point[0] << ", " << root->point[1] << std::endl;
		// if(root->left == NULL)
		// 	std::cout << "left NULL" << std::endl;
		// if(root->right == NULL)
		// 	std::cout << "right NULL" << std::endl;
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};





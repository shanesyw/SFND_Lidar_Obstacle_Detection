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

    void insert(std::vector<float> point, int id)
    {
        insertNode(&root,0,point,id);
    }

    void insertNode(Node** currentNode, int depth, std::vector<float> point, int id)
    {
        if(*currentNode == NULL)
        {
            *currentNode = new Node(point,id);
        }
        else
        {
            //get index category (2D will be 0, 1; 3D will be 0, 1, 2)
			int indexCat = depth%2; //3D will be depth%3;

			if(point[indexCat] < ((*currentNode)->point[indexCat]))
				insertNode(&((*currentNode)->left), depth+1, point, id);
			else
				insertNode(&((*currentNode)->right), depth+1, point, id);           
        }
    }


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;  // these will store index of the nodes.

		searchForTarget(&ids, root, 0, target, distanceTol);

		return ids;
	}

	void searchForTarget(std::vector<int>* NodeListPrt, Node* currentNode, int depth, std::vector<float> target, float distanceTol)
	{
		
		if(currentNode == NULL)
			return;

		//check distance of currentNode
		if(currentNode->point[0] >= target[0]-distanceTol &&
		 currentNode->point[0] <= target[0]+distanceTol && 
		 currentNode->point[1] >= target[1]-distanceTol &&
		 currentNode->point[1] <= target[1]+distanceTol )
		{
			std::cout << "within bounding box" << endl;
			// do another precise calculation
			if(sqrt(pow(target[0]-currentNode->point[0],2)+pow(target[1]-currentNode->point[1],2)) <= distanceTol)
			{
				//std::cout << "passed precise dist test" << endl;
				// insert index
				NodeListPrt->push_back(currentNode->id);
			}			
		}

		//get index category (2D will be 0, 1; 3D will be 0, 1, 2)
		int indexCat = depth%2; //3D will be depth%3;
		if(currentNode->point[indexCat]> target[indexCat]-distanceTol)
			searchForTarget(NodeListPrt, currentNode->left, depth+1, target, distanceTol);
		if(currentNode->point[indexCat]< target[indexCat]+distanceTol)
			searchForTarget(NodeListPrt, currentNode->right, depth+1, target, distanceTol);

	}


	

};





//
// Created by gatsby on 2019-02-27.
//

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

void print_query_info(octomap::point3d query, octomap::OcTreeNode *node){
	if(node != NULL)
		std::cout << "Occupancy probability at " << query << ":\t" << node->getOccupancy() << std::endl;
	else
		std::cout << "Occupancy probability at " << query << ":\t is unknown." << std::endl;
}

int main(int argc, char** argv)
{
	std::cout << std::endl;
	std::cout << "Generating example map!" << std::endl;

	octomap::OcTree tree(0.1);                                                  // create empty tree with resolution 0.1

	// insert some measurement of occupied cells.
	for(int x = -20; x < 20; x++)
		for(int y = -20; y < 20; y++)
			for(int z = -20; z < 20; z++){
				octomap::point3d endpoint((float)x*0.05f, (float)y*0.05f, (float)z*0.05f);
				tree.updateNode(endpoint, true);
			}

	for(int x = -30; x < 30; x++)
		for(int y = -30; y < 30; y++)
			for(int z = -30; z < 30; z++){
				octomap::point3d endpoint((float)x*0.02f-1.0f, (float)y*0.02f-1.0f, (float)z*0.02f-1.0f);
				tree.updateNode(endpoint, false);
			}


	std::cout << std::endl;
	std::cout << "performing some queries" << std::endl;

	octomap::point3d query(0., 0., 0.);
	octomap::OcTreeNode *result = tree.search(query);
	print_query_info(query, result);

	query = octomap::point3d(-1, -1, -1);
	result = tree.search(query);
	print_query_info(query, result);

	query = octomap::point3d(1., 1., 1.);
	result = tree.search(query);
	print_query_info(query, result);

	std::cout << std::endl;
	tree.writeBinary("simple_tree.bt");
	std::cout << "Wrote example file simple_tree.bt" << std::endl << std::endl;
}
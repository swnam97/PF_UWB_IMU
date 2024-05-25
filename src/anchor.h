/*
 * anchor.h
 */

#ifndef ANCHOR_H_
#define ANCHOR_H_
//// (?) What does it mean

#include <algorithm>

class Anchor {
public:
	
	struct single_anchors_s{

		int id_i ; // Anchor ID
		double x; // Anchor x-position in the map (global coordinates)
		double y; // Anchor y-position in the map (global coordinates)
		double z; // Anchor z-position in the map (global coordinates)
	};

    
	std::vector<single_anchors_s> anchor_list ; // List of anchors in the map

    // Function to find minimum and maximum values for x, y, z
    void findMinMaxValues(float& minX, float& maxX, float& minY, float& maxY, float& minZ, float& maxZ) const {
        if (anchor_list.empty()) {
            std::cerr << "Anchor list is empty!" << std::endl;
            return;
        }

        // Using lambda functions to find min and max for x
        auto minMaxX = std::minmax_element(anchor_list.begin(), anchor_list.end(), 
            [](const single_anchors_s& a, const single_anchors_s& b) { return a.x < b.x; });
        minX = minMaxX.first->x;
        maxX = minMaxX.second->x;

        // Using lambda functions to find min and max for y
        auto minMaxY = std::minmax_element(anchor_list.begin(), anchor_list.end(), 
            [](const single_anchors_s& a, const single_anchors_s& b) { return a.y < b.y; });
        minY = minMaxY.first->y;
        maxY = minMaxY.second->y;

        // Using lambda functions to find min and max for z
        auto minMaxZ = std::minmax_element(anchor_list.begin(), anchor_list.end(), 
            [](const single_anchors_s& a, const single_anchors_s& b) { return a.z < b.z; });
        minZ = minMaxZ.first->z;
        maxZ = minMaxZ.second->z;
    }
};



#endif /* ANCHOR_H_ */
//// (?) What is it?
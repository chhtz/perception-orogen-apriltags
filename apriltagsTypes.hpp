#ifndef apriltags_TYPES_HPP
#define apriltags_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <base/Eigen.hpp>
#include <vector>
#include <base/Time.hpp>
#include <frame_helper/Calibration.h>

namespace apriltags {

	enum family_t {TAG25H7, TAG25H9, TAG36H10, TAG36H11, TAG36ARTOOLKIT};

    struct ApriltagsConfig
	{
		ApriltagsConfig()
		{
	        debug = false;
	        quiet = false;
	        family = TAG36H11;
	        border = 1;
	        iters = 1;
	        threads = 4;
	        decimate = 1.0;
	        blur = 0.0;
	        refine_edges = true;
	        refine_decode = false;
	        refine_pose = false;
		}

        bool debug; // Enable debugging output (slow)
        bool quiet; // Reduce output
        family_t family; // Tag family to use
        int border; // Set tag family border size
        int iters; // Repeat processing on input set this many times
        int threads; // Use this many CPU threads
        double decimate; // Decimate input image by this factor
        double blur; // Apply low-pass blur to input
        bool refine_edges; // Spend more time trying to align edges of tags
        bool refine_decode; // Spend more time trying to decode tags
        bool refine_pose; // Spend more time trying to precisely localize tags
	};
	
   struct ApriltagIDToSize
   {
      int id;
      double marker_size; //in meters  
   };

    struct SingleTag
    {
        base::Vector2d corners[4];
        int id;
        bool operator<(const SingleTag& other) const { return id < other.id; }
        base::Vector2d& operator[](size_t i) {return corners[i];}
        const base::Vector2d& operator[](size_t i) const {return corners[i];}
    };

    struct TimedVectorOfTags// : public std::vector<SingleTag> // TODO why can't I inherit from std::vector?
    {
        typedef std::vector<SingleTag> Container;
        base::Time time;
        frame_helper::CameraCalibration calib;
        Container tags;
        void reserve(size_t size) { tags.reserve(size); }
        void push_back(const SingleTag& tag) { tags.push_back(tag); }
        const Container& operator*() const {return tags;}
        const Container* operator->() const {return &tags;}
        Container& operator*() {return tags;}
        Container* operator->() {return &tags;}
    };

//    struct TimedVectorOfTags : public std::vector<SingleTag> { base::Time time; };
////        std::vector<SingleTag> tags;
//    };

}



#endif


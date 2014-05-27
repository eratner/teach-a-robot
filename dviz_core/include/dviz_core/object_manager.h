#ifndef OBJECT_MANAGER_H
#define OBJECT_MANAGER_H

#include <dviz_core/object.h>
#include <dviz_core/common.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <pviz/pviz.h>
#include <pr2_collision_checker/pr2_collision_space.h>
#include <ros/package.h>
#include <sbpl_manipulation_components/shared_occupancy_grid.h>

#include <map>

namespace demonstration_visualizer 
{

class ObjectManager
{
public:
    /**
     * @brief Constructs an object manager, where the distance field is stored in shared memory.
     *        Only the DVizCore is allowed to write to shared memory, so unless core is true,
     *        the object manager instance will only be able to read distance fields from shared 
     *        memory.
     */
    ObjectManager(std::string rarm_file, std::string larm_file, int user_id = 0, bool core = false);

    ~ObjectManager();

    /**
     * @brief Initialize the collision checker with the given dimensions and origin.
     *        Also, the provided name is used to reference a precomputed distance field
     *        stored in shared memory which can alternatively be used to initialize the
     *        collision checker. If this is the core, then a distance field with that 
     *        name will be placed in shared memory.
     */
    bool initializeCollisionChecker(const std::vector<double> &dims, 
				    const std::vector<double> &origin,
				    const std::string &name = "");
    void addObject(Object o);
    
    bool addObjectFromFile(visualization_msgs::Marker &mesh_marker,
			   const std::string &collision_model_file, 
			   bool movable = false);

    void removeObject(int id);
    void clearObjects();
    std::vector<visualization_msgs::Marker> getMovedMarkers();
    std::vector<visualization_msgs::Marker> getMarkers();
    visualization_msgs::Marker getMarker(int id);

    bool checkRobotMove(const std::vector<double> &rangles, 
			const std::vector<double> &langles, 
			BodyPose &bp, int skip_id = -1);

    bool checkObjectMove(int id, geometry_msgs::Pose p,
                         std::vector<double> rangles, std::vector<double> langles, BodyPose bp);
    void moveObject(int id, const geometry_msgs::Pose &p);

    void scaleObject(int id, double x, double y, double z);

    int getNumObjects() const;

    std::vector<Object> getObjects() const;

    std::string getObjectLabel(int id);

    bool addObjectsFromOccupiedVoxelsFile(const std::string &filename);

    bool writeObjectsToOccupiedVoxelsFile(const std::string &filename);

    void visualizeObjectCollisionModels();

    std::vector<double> getBoundingBoxDimensions() const;

    std::vector<double> getBoundingBoxOrigin() const;

    bool initSharedDistanceField();

  private:
    int user_id_;
    bool is_core_;

    sbpl_arm_planner::SharedOccupancyGrid *shared_occupancy_grid_;
    pr2_collision_checker::PR2CollisionSpace* collision_checker_;
    std::map<int, Object> objects_;
    std::string rarm_file_;
    std::string larm_file_;
    bool enable_debug_visualizations_;
    bool disable_collision_checking_;
    bool load_objects_from_voxels_file_;
    bool visualize_collision_models_;
    std::vector<double> bounding_box_dimensions_;
    std::vector<double> bounding_box_origin_;

};

} // namespace demonstration_visualizer

#endif

#ifndef RRTSTAR_SEARCHER_H
#define RRTSTAR_SEARCHER_H

#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <unordered_map>
#include <set>

using namespace Eigen;
using namespace std;

struct MappingNode {
    Vector3i index;
    Vector3d coord;
    double g_score;
    double f_score;
    int id;
    MappingNode* Father;

    MappingNode(Vector3i _index, Vector3d _coord) : index(_index), coord(_coord), g_score(DBL_MAX), f_score(DBL_MAX), id(0), Father(nullptr) {}
};

typedef MappingNode* MappingNodePtr;

class RRTstarpath {
public:
    RRTstarpath();
    ~RRTstarpath();

    void begin_grid_map(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
    void resetGrid(MappingNodePtr ptr);
    void resetUsedGrids();
    void set_barrier(const double coord_x, const double coord_y, const double coord_z);
    vector<Vector3d> getVisitedNodes();
    Vector3d gridIndex2coord(const Vector3i &index);
    Vector3i coord2gridIndex(const Vector3d &pt);
    Eigen::Vector3d coordRounding(const Eigen::Vector3d &coord);
    inline bool isOccupied(const Eigen::Vector3i &index) const;
    inline bool isFree(const Eigen::Vector3i &index) const;
    bool RRTstarSearch(Vector3d start_pt, Vector3d end_pt);
    vector<Vector3d> getPath();
    std::vector<Vector3d> pathSimplify(const vector<Vector3d> &path, double path_resolution);
    double perpendicularDistance(const Eigen::Vector3d point_insert, const Eigen::Vector3d point_st, const Eigen::Vector3d point_end);
    Vector3d getPosPoly(MatrixXd polyCoeff, int k, double t);
    int safeCheck(MatrixXd polyCoeff, VectorXd time);
    void resetOccupy();

private:
    double resolution, inv_resolution;
    Vector3d gl_xl, gl_yl, gl_zl, gl_xu, gl_yu, gl_zu;
    int GRID_X_SIZE, GRID_Y_SIZE, GRID_Z_SIZE, GLYZ_SIZE, GLXYZ_SIZE;
    uint8_t *data;
    MappingNodePtr ***Map_Node;
    Vector3i goalIdx;
    MappingNodePtr terminatePtr;
    multimap<double, MappingNodePtr> Openset;

    void RRTstarGetSucc(MappingNodePtr currentPtr, vector<MappingNodePtr> &neighborPtrSets, vector<double> &edgeCostSets);
    double getHeu(MappingNodePtr node1, MappingNodePtr node2);
    bool isOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const;
    bool isFree(const int &idx_x, const int &idx_y, const int &idx_z) const;
};

#endif // RRTSTAR_SEARCHER_H
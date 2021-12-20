#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;
bool t_b = false;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l,
                                  Vector3d global_xyz_u, int max_x_id,
                                  int max_y_id, int max_z_id) {
  gl_xl = global_xyz_l(0);
  gl_yl = global_xyz_l(1);
  gl_zl = global_xyz_l(2);

  gl_xu = global_xyz_u(0);
  gl_yu = global_xyz_u(1);
  gl_zu = global_xyz_u(2);

  GLX_SIZE = max_x_id;
  GLY_SIZE = max_y_id;
  GLZ_SIZE = max_z_id;
  GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
  GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

  resolution = _resolution;
  inv_resolution = 1.0 / _resolution;

  data = new uint8_t[GLXYZ_SIZE];
  memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));

  GridNodeMap = new GridNodePtr **[GLX_SIZE];
  for (int i = 0; i < GLX_SIZE; i++) {
    GridNodeMap[i] = new GridNodePtr *[GLY_SIZE];
    for (int j = 0; j < GLY_SIZE; j++) {
      GridNodeMap[i][j] = new GridNodePtr[GLZ_SIZE];
      for (int k = 0; k < GLZ_SIZE; k++) {
        Vector3i tmpIdx(i, j, k);
        Vector3d pos = gridIndex2coord(tmpIdx);
        GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
      }
    }
  }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr) {
  ptr->id = 0;
  ptr->cameFrom = NULL;
  ptr->gScore = inf;
  ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids() {
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++)
        resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y,
                             const double coord_z) {
  if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
      coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
    return;

  int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
  int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
  int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

  if (idx_x == 0 || idx_y == 0 || idx_z == GLZ_SIZE || idx_x == GLX_SIZE ||
      idx_y == GLY_SIZE)
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
  else {
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
  }
}

vector<Vector3d> AstarPathFinder::getVisitedNodes() {
  vector<Vector3d> visited_nodes;
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++) {
        // if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and
        // close list
        if (GridNodeMap[i][j][k]->id ==
            -1) // visualize nodes in close list only
          visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
      }

  ROS_WARN("visited_nodes size : %d", visited_nodes.size());
  return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i &index) {
  Vector3d pt;

  pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
  pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

  return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d &pt) {
  Vector3i idx;
  idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
      min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
      min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

  return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d &coord) {
  return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i &index) const {
  return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i &index) const {
  return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int &idx_x, const int &idx_y,
                                        const int &idx_z) const {
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int &idx_x, const int &idx_y,
                                    const int &idx_z) const {
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr,
                                          vector<GridNodePtr> &neighborPtrSets,
                                          vector<double> &edgeCostSets) {
  neighborPtrSets.clear();
  edgeCostSets.clear();
  Vector3i neighborIdx;
  GridNodePtr temp_ptr = nullptr;
  Eigen::Vector3d n_coord;
  double dist;
  auto this_coord = currentPtr -> coord;
  for (int dx = -1; dx < 2; dx++) {
    for (int dy = -1; dy < 2; dy++) {
      for (int dz = -1; dz < 2; dz++) {

        if (dx == 0 && dy == 0 && dz == 0)
          continue;

        neighborIdx(0) = (currentPtr->index)(0) + dx;
        neighborIdx(1) = (currentPtr->index)(1) + dy;
        neighborIdx(2) = (currentPtr->index)(2) + dz;

        if (neighborIdx(0) < 0 || neighborIdx(0) >= GLX_SIZE ||
            neighborIdx(1) < 0 || neighborIdx(1) >= GLY_SIZE ||
            neighborIdx(2) < 0 || neighborIdx(2) >= GLZ_SIZE) {
          continue;
        }
        if(isOccupied(neighborIdx))
                    continue; // skip obstacles
        // put the pointer into neighborPtrSets
        temp_ptr = GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
        if(temp_ptr->id == -1) 
            continue; // skip
        n_coord = temp_ptr->coord;
        dist = std::sqrt( (n_coord[0] - this_coord[0]) * (n_coord[0] - this_coord[0])+
                        (n_coord[1] - this_coord[1]) * (n_coord[1] - this_coord[1])+
                        (n_coord[2] - this_coord[2]) * (n_coord[2] - this_coord[2]));

        neighborPtrSets.push_back(
            GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)]);
        edgeCostSets.push_back(sqrt(dx * dx + dy * dy + dz * dz));
      }
    }
  }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2) {
  // using digonal distance and one type of tie_breaker.
  double h;
	
  auto node1_coord = node1->coord;
  auto node2_coord = node2->coord;
//===========================================================
  // Manhattan
  // h = std::abs(node1_coord(0) - node2_coord(0) ) + std::abs(node1_coord(1) - node2_coord(1) ) + std::abs(node1_coord(2) - node2_coord(2) );
//===========================================================
  // Euclidean
  // h = std::sqrt(std::pow((node1_coord(0) - node2_coord(0)), 2 ) + std::pow((node1_coord(1) - node2_coord(1)), 2 ) + std::pow((node1_coord(2) - node2_coord(2)), 2 ));
//===========================================================
  // Diagonal distance
  double dx = std::abs(node1_coord(0) - node2_coord(0) );
  double dy = std::abs(node1_coord(1) - node2_coord(1) );
  double dz = std::abs(node1_coord(2) - node2_coord(2) );
  double diagonal = std::min({dx, dy, dz});
  // h =std::sqrt(2.0)*diagonal + (dx + dy + dz- 2*diagonal);
  h = dx + dy + dz + (std::sqrt(3.0) -3) * diagonal; 

  if(t_b){
    double p = 1.0 / 25.0;
    h *= (1.0 + p);
  }
//=======================================================
  return h;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt) {
  ros::Time time_1 = ros::Time::now();

  // index of start_point and end_point
  Vector3i start_idx = coord2gridIndex(start_pt);
  // cout << "start_pt"<< start_pt << endl;
  // cout << "start_idx"<< start_idx << endl;
  Vector3i end_idx = coord2gridIndex(end_pt);
  goalIdx = end_idx;

  // position of start_point and end_point
  start_pt = gridIndex2coord(start_idx);
  // cout << "start_pt"<< start_pt << endl;
  // cout << "start_idx"<< start_idx << endl;
  end_pt = gridIndex2coord(end_idx);

  // Initialize the pointers of struct GridNode which represent start node and
  // goal node
  GridNodePtr startPtr = new GridNode(start_idx, start_pt);
  GridNodePtr endPtr = new GridNode(end_idx, end_pt);

  cout << "real start_pt"<< start_pt << endl;
  cout << "real end_pt"<< end_pt << endl;

  // openSet is the open_list implemented through multimap in STL library
  openSet.clear();
  // currentPtr represents the node with lowest f(n) in the open_list
  GridNodePtr currentPtr = NULL;
  GridNodePtr neighborPtr = NULL;

  // put start node in open set
  startPtr->gScore = 0;
  /**
   *
   * STEP 1.1:  finish the AstarPathFinder::getHeu
   *
   * **/
  startPtr->fScore = getHeu(startPtr, endPtr);

  startPtr->id = 1;
  startPtr->coord = start_pt;
  openSet.insert(make_pair(startPtr->fScore, startPtr));

  /**
   *
   * STEP 1.2:  some else preparatory works which should be done before while
   * loop
   *
   * **/
  Eigen::Vector3i current_idx;
  GridNodeMap[start_idx[0]][start_idx[1]][start_idx[2]] -> id = 1; //

  double tentative_gScore;
  vector<GridNodePtr> neighborPtrSets;
  vector<double> edgeCostSets;

  /**
   *
   * STEP 1.3:  finish the loop
   *
   * **/
  while (!openSet.empty()) {
	  currentPtr = openSet.begin() -> second; // 
    openSet.erase(openSet.begin()); // remove the node with lowest f_cost
    current_idx = currentPtr->index;
    GridNodeMap[current_idx[0]][current_idx[1]][current_idx[2]] -> id = -1;// update the id in grid node map
    // cout << "current_idx"<< current_idx << endl;
    // if the current node is the goal 
    if( currentPtr->index == goalIdx ){
        ros::Time time_2 = ros::Time::now();
        terminatePtr = currentPtr;
        ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
        return;
    }
    //get the succetion
    AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);
    for(int i = 0; i < (int)neighborPtrSets.size(); i++){
      /*
      *
      *
      Judge if the neigbors have been expanded
      please write your code below
      
      IMPORTANT NOTE!!!
      neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
      neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open set
      *        
      */
      neighborPtr = neighborPtrSets[i];
      if(neighborPtr -> id == 0){ //discover a new node, which is not in the closed set and open set
        /*
        *
        *
        As for a new node, do what you need do ,and then put neighbor in open set and record it
        please write your code below
        *        
        */
        neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
        neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr,endPtr);
        neighborPtr->cameFrom = currentPtr; // todo shallow copy or deep copy
        // push node "m" into OPEN
        openSet.insert(make_pair(neighborPtr -> fScore, neighborPtr));
        neighborPtr -> id = 1;
        // cout << "neighborPtr -> id = 0" << endl;
        continue;
        
      }
      else if(neighborPtr -> id == 1){ //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
        /*
        *
        *
        As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
        please write your code below
        *        
        */
        //update: g_c, f_c, cameFrom
        if(neighborPtr -> gScore > (currentPtr -> gScore + edgeCostSets[i])){
          neighborPtr -> gScore = currentPtr -> gScore + edgeCostSets[i];
          neighborPtr -> fScore = neighborPtr -> gScore + getHeu(neighborPtr,endPtr);
          neighborPtr -> cameFrom = currentPtr;
        }
        // cout << "neighborPtr -> id = 1" << endl;
        continue;
      }
      else{//this node is in closed set
        /*
        *
        please write your code below
        *        
        */
        // cout << "closed set" << endl;
        continue;
      }
    }
  }
  cout << "========================================================================================" << endl;
  cout << "========================================================================================" << endl;
  // if search fails
  ros::Time time_2 = ros::Time::now();
  if ((time_2 - time_1).toSec() > 0.1)
  {
    // ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec());
    cout << "========================================================================================" << endl;
    cout << "Time consume in Astar path finding is"<< (time_2 - time_1).toSec() << endl;
    cout << "========================================================================================" << endl;
  }
    

}

vector<Vector3d> AstarPathFinder::getPath() {
  vector<Vector3d> path;
  vector<GridNodePtr> gridPath;

  /**
   *
   * STEP 1.4:  trace back the found path
   *
   * **/
  auto ptr = terminatePtr;
  if(terminatePtr == NULL)
  {
    cout << "========================================================================================" << endl;
    cout << "========================================================================================" << endl;
    cout << "terminatePtr == NULL" << endl;
    cout << "========================================================================================" << endl;
    cout << "========================================================================================" << endl;
    return path;
  }
  while(ptr -> cameFrom != NULL){
    gridPath.push_back(ptr);
    ptr = ptr->cameFrom;
  }

  for (auto ptr: gridPath)
    path.push_back(ptr->coord);
      
  reverse(path.begin(),path.end());

  return path;
}

double AstarPathFinder::perpendicularDistance(const Eigen::Vector3d & p, const Eigen::Vector3d & p_start, const Eigen::Vector3d & p_end)
{
  Eigen::Vector3d vec1 = p-p_start;//Eigen::Vector3d(p(0) - p_start(0), p(1) - p_start(1), p(2) - p_start(3));
	Eigen::Vector3d vec2 = p_end-p_start;//Eigen::Vector3d(p_end(0) - p_start(0), p_end(1) - p_start(1), p_end(2) - p_start(2));
	Eigen::Vector3d vec3 = vec1.cross(vec2);
  double d_vec2 = vec2.norm();//sqrt(vec2(0)*vec2(0) + vec2(1)*vec2(1));
	double cross_product = vec3.norm();//vec1(0)*vec2(1) - vec2(0)*vec1(1);
	double d = 0;
  if(d_vec2>1e-8)
    d = abs(cross_product / d_vec2);
  else
    d = vec1.norm();
	return d;
}

vector<Vector3d> AstarPathFinder::pathSimplify(const vector<Vector3d> &path,
                                               double path_resolution) {
  vector<Vector3d> subPath;
  /**
   *
   * STEP 2.1:  implement the RDP algorithm
   *
   * **/
	// Find the point with the maximum distance
	double dmax = 0;
	int index = 0;

	for (int i = 1; i < path.size() - 1; ++i)
	{

		double d = perpendicularDistance(path[i], path[0], path[path.size() - 1]);
		if (d > dmax) {
			index = i;
			dmax = d;
		}
  }
	// If max distance is greater than epsilon, recursively simplify
  if (dmax > path_resolution)
  {
    std::vector<Eigen::Vector3d> pre_part, next_part;
    for (int i = 0; i <= index; ++i)	pre_part.push_back(path[i]);
    for (int i = index; i < path.size(); ++i)	next_part.push_back(path[i]);
    // Recursive call
    std::vector<Eigen::Vector3d> subPath1 = pathSimplify(pre_part, path_resolution);
    std::vector<Eigen::Vector3d> subPath2 = pathSimplify(next_part, path_resolution);

    // combine
    subPath.insert(subPath.end(), subPath1.begin(), subPath1.end());
    subPath.insert(subPath.end(), subPath2.begin()+1, subPath2.end());
  }
  else
  {
    subPath.push_back(path[0]);
    subPath.push_back(path[path.size() - 1]);
  }
  for (int i = 0; i < path.size(); ++i)
	{
    // cout << "path:" << path[i] << endl;
  }
  for (int i = 0; i < subPath.size(); ++i)
	{
    // cout << "subPath:" << subPath[i] << endl;
  }
  return subPath;
}

Vector3d AstarPathFinder::getPosPoly(MatrixXd polyCoeff, int k, double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}

int AstarPathFinder::safeCheck(MatrixXd polyCoeff, VectorXd time) {
  int unsafe_segment = -1; //-1 -> the whole trajectory is safe
  /**
   *
   * STEP 3.3:  finish the sareCheck()
   *
   * **/
  

  Vector3d pos = Vector3d::Zero();
  for (int i = 0; i < time.size(); i++) {
    for (double t = 0.0; t < time(i); t += 0.01) {
      pos = getPosPoly(polyCoeff, i, t);

      double d = perpendicularDistance(pos, getPosPoly(polyCoeff, i, 0.0), getPosPoly(polyCoeff, i, time(i)));
      if (d > 0.2) {
        unsafe_segment = i;
      }
      // Vector3i pos_idx = coord2gridIndex(pos);
      // if(isOccupied(pos_idx))
      // {
      //   unsafe_segment = i;
      // }
    }
  }
  
  return unsafe_segment;
}
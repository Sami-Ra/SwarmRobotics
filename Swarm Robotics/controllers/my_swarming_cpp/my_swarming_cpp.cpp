// File:          my_swarming_cpp.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <array>
#include <algorithm>
#include <map>
#include <list>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <complex>
#include <cmath>
#include <chrono>
#include <iomanip>

#include <webots/Robot.hpp>
#include <webots/supervisor.hpp>
#include <webots/motor.hpp>
#include <webots/gps.hpp>
#include <webots/compass.hpp>
#include <webots/emitter.hpp>
#include <webots/receiver.hpp>
#include <webots/Node.hpp>
#include <webots/keyboard.hpp>

#define N_FLOCKS 10

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

#define MAX_SPEED 6.28 //angular speed in rad/s


#define GPS_SAMPLING_PERIOD 1 //in ms
#define COMPASS_SAMPLING_PERIOD 1 //in ms

#define COMMUNICATION_CHANNEL 0
#define K_Kmeans 3 
double dist_threshold=0.3;
// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

int process_started=0;
class Point {

private:
    int pointId, clusterId;
    int dimensions;
    vector<double> values;

public:
    Point(int id, double x, double y) {
        dimensions = 0;
        pointId = id;
        values.push_back(x);
        dimensions++;
        values.push_back(y);
        dimensions++;
        clusterId = 0; //Initially not assigned to any cluster
    }

    int getDimensions() {
        return dimensions;
    }

    int getCluster() {
        return clusterId;
    }

    int getID() {
        return pointId;
    }

    void setCluster(int val) {
        clusterId = val;
    }

    double getVal(int pos) {
        return values[pos];
    }
};

class Cluster {

private:
    int clusterId;
    vector<double> centroid;
    vector<Point> points;

public:
    Cluster(int clusterId, Point centroid) {
        this->clusterId = clusterId;
        for (int i = 0; i < centroid.getDimensions(); i++) {
            this->centroid.push_back(centroid.getVal(i));
        }
        this->addPoint(centroid);
    }

    void addPoint(Point p) {
        p.setCluster(this->clusterId);
        points.push_back(p);
    }

    bool removePoint(int pointId) {
        int size = points.size();

        for (int i = 0; i < size; i++)
        {
            if (points[i].getID() == pointId)
            {
                points.erase(points.begin() + i);
                return true;
            }
        }
        return false;
    }

    int getId() {
        return clusterId;
    }

    Point getPoint(int pos) {
        return points[pos];
    }

    int getSize() {
        return points.size();
    }

    double getCentroidByPos(int pos) {
        return centroid[pos];
    }

    void setCentroidByPos(int pos, double val) {
        this->centroid[pos] = val;
    }
};

class KMeans {
private:
    int K, iters, dimensions, total_points;
    vector<Cluster> clusters;

    int getNearestClusterId(Point point) {
        double sum = 0.0, min_dist;
        int NearestClusterId;

        for (int i = 0; i < dimensions; i++)
        {
            sum += pow(clusters[0].getCentroidByPos(i) - point.getVal(i), 2.0);
        }

        min_dist = sqrt(sum);
        NearestClusterId = clusters[0].getId();

        for (int i = 1; i < K; i++)
        {
            double dist;
            sum = 0.0;

            for (int j = 0; j < dimensions; j++)
            {
                sum += pow(clusters[i].getCentroidByPos(j) - point.getVal(j), 2.0);
            }

            dist = sqrt(sum);

            if (dist < min_dist)
            {
                min_dist = dist;
                NearestClusterId = clusters[i].getId();
            }
        }

        return NearestClusterId;
    }

public:
    KMeans(int K, int iterations) {
        this->K = K;
        this->iters = iterations;
    }

    vector<Cluster> run(vector<Point>& all_points) {

        total_points = all_points.size();
        dimensions = all_points[0].getDimensions();


        //Initializing Clusters
        vector<int> used_pointIds;

        for (int i = 1; i <= K; i++)
        {
            while (true)
            {
                int index = rand() % total_points;

                if (find(used_pointIds.begin(), used_pointIds.end(), index) == used_pointIds.end())
                {
                    used_pointIds.push_back(index);
                    all_points[index].setCluster(i);
                    Cluster cluster(i, all_points[index]);
                    clusters.push_back(cluster);
                    break;
                }
            }
        }
        cout << "Clusters initialized = " << clusters.size() << endl << endl;


        cout << "Running K-Means Clustering.." << endl;

        int iter = 1;
        while (true)
        {
            cout << "Iter - " << iter << "/" << iters << endl;
            bool done = true;

            // Add all points to their nearest cluster
            for (int i = 0; i < total_points; i++)
            {
                int currentClusterId = all_points[i].getCluster();
                int nearestClusterId = getNearestClusterId(all_points[i]);

                if (currentClusterId != nearestClusterId)
                {
                    if (currentClusterId != 0) {
                        for (int j = 0; j < K; j++) {
                            if (clusters[j].getId() == currentClusterId) {
                                clusters[j].removePoint(all_points[i].getID());
                            }
                        }
                    }

                    for (int j = 0; j < K; j++) {
                        if (clusters[j].getId() == nearestClusterId) {
                            clusters[j].addPoint(all_points[i]);
                        }
                    }
                    all_points[i].setCluster(nearestClusterId);
                    done = false;
                }
            }

            // Recalculating the center of each cluster
            for (int i = 0; i < K; i++)
            {
                int ClusterSize = clusters[i].getSize();

                for (int j = 0; j < dimensions; j++)
                {
                    double sum = 0.0;
                    if (ClusterSize > 0)
                    {
                        for (int p = 0; p < ClusterSize; p++)
                            sum += clusters[i].getPoint(p).getVal(j);
                        clusters[i].setCentroidByPos(j, sum / ClusterSize);
                    }
                }
            }

            if (done || iter >= iters)
            {
                cout << "Clustering completed in iteration : " << iter << endl << endl;
                break;
            }
            iter++;
        }
        return clusters;
    }
};

class Graph {
    // A function used by DFS
    void DFSUtil(int v);
public:
    map<int, bool> visited;
    map<int, list<int>> adj;
    int n_parts = 0;
    // function to add an edge to graph
    void addEdge(int v, int w);
    // prints DFS traversal of the complete graph
    void DFS();
};

void Graph::addEdge(int v, int w)
{
    adj[v].push_back(w); // Add w to v’s list.
}

void Graph::DFSUtil(int v)
{
    // Mark the current node as visited and print it
    visited[v] = true;
    // Recur for all the vertices adjacent to this vertex
    list<int>::iterator i;
    for (i = adj[v].begin(); i != adj[v].end(); ++i)
        if (!visited[*i])
            DFSUtil(*i);
}

// The function to do DFS traversal. It uses recursive
void Graph::DFS()
{
    // Call the recursive helper function to print DFS
    // traversal starting from all vertices one by one
    for (auto i : adj)
        if (visited[i.first] == false) {
            n_parts++;
            DFSUtil(i.first);
        }
}


 


Keyboard *keyboard;

struct msg{
  int robot_no;
  double x,y,z;
}msg;

class my_swarming : public Supervisor{
//protected:
public:
  my_swarming();
  string robot_def[N_FLOCKS];
  double cur_loc[N_FLOCKS][4];
  double dest_loc[N_FLOCKS][2];
  int timeStep;

  Node* robot_node[N_FLOCKS];
  Node* target_node[N_FLOCKS];  
  Emitter* emitter;  
  vector<Cluster> clusters;
  void computeCurLocs();
  double computeDot(double x[3],double y[3]);
  double computeDist(double x[3],double y[3]);
  double dist_threshold;
  void Alignment();
  void Cohesion();
  double measureCohesion();
  int measureAlignment();
  void measureFitnessStep();
  void sendMsg(int,double[4]);
  double maxCohesion = 0;
  //const double *target_translation;
};

void my_swarming::computeCurLocs(){
  Field* trans_field;
  for (int i=0 ; i<N_FLOCKS ; i++){
    trans_field = robot_node[i]->getField("translation");
    
    const double* loc= trans_field->getSFVec3f();
    cur_loc[i][0]=loc[0]; 
    cur_loc[i][1]=loc[1];
    cur_loc[i][2]=loc[2]; 
    cur_loc[i][3]=loc[3];
   }
}

void my_swarming::measureFitnessStep()
{
  double cohesion = measureCohesion();
  int alignment = measureAlignment();
  
  auto now = std::chrono::system_clock::now();
  auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
  auto value = now_ms.time_since_epoch();
  long duration = value.count();
  long milliseconds = duration % 1000;
  if(maxCohesion < cohesion)
    maxCohesion = cohesion;
  else if(process_started)
    process_started = 2;
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);

  // Create a stringstream to store the formatted time
  std::stringstream ss;
  ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S.") << std::setw(3) << std::setfill('0') << milliseconds;

  // Copy the formatted time string to a std::string
  std::string formatted_time = ss.str();

  // Print the formatted time string
  std::cout << formatted_time << std::endl;  
  
  cout << "cohesion:" << cohesion << " maxCohesion: "<< maxCohesion <<endl;
  cout << "alignmant:" << alignment << endl;

  std::string filename = "evaluate.csv";
  std::ofstream data(filename, std::ios_base::app);
  if (data.is_open()) {
    data << cohesion<<','<<alignment<<','<<formatted_time<<endl;
  }
  data.close();
}


int my_swarming::measureAlignment()
{
  Graph g;
  double dist;
  computeCurLocs();
  double loc1[3],loc2[3];
  for (int i=0;i<N_FLOCKS;i++)
    for (int j=0;j<N_FLOCKS;j++){
    
      loc1[0] = cur_loc[i][0]; loc1[1] = cur_loc[i][1] ; loc1[2] = cur_loc[i][2] ; 
      loc2[0] = cur_loc[j][0]; loc2[1] = cur_loc[j][1] ; loc2[2] = cur_loc[j][2] ; 
  
      dist=computeDist(loc1,loc2);
      if (dist<dist_threshold){
        g.addEdge(i, j);
        g.addEdge(j, i);
      }
     }
  g.DFS();
  return g.n_parts;  
}

double my_swarming::measureCohesion()
{
  // The center of mass is already computed in the main function
  // Compute the average distance to the center of mass
  double dist = 0.0;
  double loc[3],centerOfMass[3]={0,0,0};


  computeCurLocs();
  for(int i=0 ; i<N_FLOCKS ; ++i)
  {
    centerOfMass[0]+=cur_loc[i][0];
    centerOfMass[2]+=cur_loc[i][2];
  }
  centerOfMass[0]/=N_FLOCKS;
  centerOfMass[2]/=N_FLOCKS;
  
  for(int i=0 ; i<N_FLOCKS ; ++i)
  {
    loc[0] = cur_loc[i][0]; loc[1] = cur_loc[i][1] ; loc[2] = cur_loc[i][2] ; 
    cout<<"loc:"<<loc[0]<<' '<<loc[2]<<endl;
    dist += computeDist(centerOfMass, loc); // Thanks to the dirty C syntax, loc[i][4] is automatically "cast" into loc[i][3]
  }
  dist /= N_FLOCKS;

  // Return the metric
  return 1.0/(1.0+dist); // The closest, the better is
}


void my_swarming::sendMsg(int robot_no,double loc[4]){
  msg.robot_no=robot_no;
  msg.x = loc[0];
  msg.y = loc[1];
  msg.z = -loc[2];
  emitter->send((void*)&msg,sizeof(msg));
}

void my_swarming::Alignment(){
  int K= K_Kmeans;
  int pointId = 0;
  vector<Point> all_points;

    
  Field* trans_field[N_FLOCKS];
  for (int i=0 ; i<N_FLOCKS ; i++){
    trans_field[i] = robot_node[i]->getField("translation");
    
    const double* loc= trans_field[i]->getSFVec3f();
    cur_loc[i][0]=loc[0]; 
    cur_loc[i][1]=loc[1];
    cur_loc[i][2]=loc[2]; 
    cur_loc[i][3]=loc[3];
    
    Point point(pointId, loc[0],loc[2]);
    all_points.push_back(point);
    pointId++;
    
  }

    if (all_points.size() < K) {
        cout << "Error: Number of clusters greater than number of points." << endl;
        exit(1);
    }

    //Running K-Means Clustering
    int iters = 100;

    KMeans kmeans(K, iters);
    clusters = kmeans.run(all_points);

    //Print pointIds in each cluster
    double center[3];
    for (int i = 0; i < K; i++) {
        center[0]= clusters[i].getCentroidByPos(0);
        center[1]= 0;
        center[2]= clusters[i].getCentroidByPos(1);
        
        cout << "e-pucks in cluster " << clusters[i].getId() << " : ";
        for (int j = 0; j < clusters[i].getSize(); j++) {
            int e_puck = clusters[i].getPoint(j).getID();
            cout << e_puck << " ";
            sendMsg(e_puck,center);
        }
        cout << endl << endl;
    }
    cout << "========================" << endl << endl;
    
    int dimensions = all_points[0].getDimensions();
    for (int i = 0; i < K; i++) {
        cout << "Cluster " << clusters[i].getId() << " centroid : ";
        for (int j = 0; j < dimensions; j++) {
            cout << clusters[i].getCentroidByPos(j) << " ";     //Output to console
        }
        cout << endl;
    }
}

void my_swarming::Cohesion(){
    double globalCenter[3]={0,0,0};
    for (int i = 0; i < K_Kmeans; i++) {
        //cout << "Cluster " << clusters[i].getId() << " centroid : ";
        //for (int j = 0; j < dimensions; j++) {
        globalCenter[0] += clusters[i].getCentroidByPos(0) ;     //Output to console
        globalCenter[2] += clusters[i].getCentroidByPos(1) ;     //Output to console
        //}
    }
    globalCenter[0]/= K_Kmeans;
    globalCenter[2]/= K_Kmeans;
    cout<<"Global center in my_swarming\n";
    for (int i = 0; i < N_FLOCKS; i++) 
      sendMsg(i,globalCenter);
}


double my_swarming::computeDist(double x[3],double y[3])
{
  return std::sqrt((x[0]-y[0]) * (x[0]-y[0]) +
                   (x[1]-y[1]) * (x[1]-y[1]) +
                   (x[2]-y[2]) * (x[2]-y[2]));
}

/*
 * Compute the dot product
 */
double my_swarming::computeDot(double x[3],double y[3])
{
  return x[0]*y[0] + x[1]*y[1] + x[2]*y[2];
}


my_swarming :: my_swarming(){
  dist_threshold=1.0;
  timeStep = 128;
  keyboard = getKeyboard();
  keyboard->enable(timeStep);

  //emitter = wb_supervisor_node_get_from_def("EMITTER");
  emitter = getEmitter("emitter");
  emitter->setChannel(COMMUNICATION_CHANNEL);
  double range = emitter->getRange();
  Field* trans_field[N_FLOCKS];


  for (int i=0 ; i<N_FLOCKS ; i++){
    
    robot_def[i]="ep"+to_string(i);
    robot_node[i] =  getFromDef(robot_def[i]);
    if (robot_node[i] == NULL) {
      cout<<"supervosor: error: can not get robot "<<robot_def[i];
      exit(1);
    }
    
    trans_field[i] = robot_node[i]->getField("translation");
    
    const double* loc= trans_field[i]->getSFVec3f();
    cur_loc[i][0]=loc[0]; 
    cur_loc[i][1]=loc[1];
    cur_loc[i][2]=loc[2]; 
    cur_loc[i][3]=loc[3];    
  }

  std::string filename = "evaluate.csv";
  std::ofstream data(filename, std::ios::out);
  if (data.is_open()) {
    data << "Cohesion,Alignment,Time\n";
  }
  data.close();

}

int main(int argc, char **argv) {
  // create the Robot instance.
  my_swarming my_swarming;
  Field* trans_field[N_FLOCKS];
  
  // get the time step of the current world.
  process_started = 0;
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (my_swarming.step(my_swarming.timeStep) != -1) {
      int k = keyboard->getKey();
      switch (k) {
        case 'A':
        case 'a':
          cout<<" Swarming Alignment started\n";
          my_swarming.Alignment();
          process_started=1;
          break;
        case 'I':
        case 'i':
          for (int i=0 ; i<N_FLOCKS ; i++){
            trans_field[i] = my_swarming.robot_node[i]->getField("translation");
            const double* loc= trans_field[i]->getSFVec3f();
            my_swarming.cur_loc[i][0]=loc[0]; 
            my_swarming.cur_loc[i][1]=loc[1];
            my_swarming.cur_loc[i][2]=loc[2]; 
            my_swarming.cur_loc[i][3]=loc[3];
            my_swarming.sendMsg(i,my_swarming.cur_loc[i]);
            cout<<"supervisor: message to send: robot no: "<< msg.robot_no<< "current loc: ("<< msg.x<<','<<msg.y<<','<<msg.z<<")\n";
          }
          break;
        case 'C':
        case 'c':
          my_swarming.Cohesion();
        }
        if(process_started==2){ 
           
          my_swarming.Cohesion();
         }
    if (process_started)
      my_swarming.measureFitnessStep();
    
  }

  // Enter here exit cleanup code.
  return 0;
}

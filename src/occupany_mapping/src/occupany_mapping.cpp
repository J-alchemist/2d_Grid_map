#include "occupany_mapping.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point32.h"

/**
 * 2D画线算法　来进行计算两个点之间的grid cell
 * 计算这两个点连线之间的cell的index
 * Bresenham
 * */
std::vector<GridIndex> TraceLine(int x0, int y0, int x1, int y1)
{
  GridIndex tmpIndex;
  std::vector<GridIndex> gridIndexVector;

  bool steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep)
  {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  if (x0 > x1)
  {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  int deltaX = x1 - x0;
  int deltaY = abs(y1 - y0);
  int error = 0;
  int ystep;
  int y = y0;

  if (y0 < y1)
  {
    ystep = 1;
  }
  else
  {
    ystep = -1;
  }

  int pointX;
  int pointY;
  for (int x = x0; x <= x1; x++)
  {
    if (steep)
    {
      pointX = y;
      pointY = x;
    }
    else
    {
      pointX = x;
      pointY = y;
    }

    error += deltaY;

    if (2 * error >= deltaX)
    {
      y += ystep;
      error -= deltaX;
    }

    //不包含最后一个点．
    if(pointX == x1 && pointY == y1) continue;

    //保存所有的点
    tmpIndex.SetIndex(pointX,pointY);

    gridIndexVector.push_back(tmpIndex);
  }

  return gridIndexVector;
}

void SetMapParams(void )
{ 
    // 36m × 36m
   mapParams.width = 900;   // 单位：像素pixel    ---> 地图实际是 900x0.04 = 36m长度
   mapParams.height = 900;
   mapParams.resolution = 0.04; // 单位：m/pixel

   //每次被击中的log变化值. 
   mapParams.log_free = -1;
   mapParams.log_occ = 2;

   //每个栅格的最大最小值．
   mapParams.log_max = 100.0;
   mapParams.log_min = 0.0;

  // 左下角（0,0）像素坐标系的原点在地图坐标系的坐标
   mapParams.origin_x = 0;
   mapParams.origin_y = 0;

   //调整像素坐标的位置
   mapParams.offset_x = 700;		// width--x
   mapParams.offset_y = 300;//600	// height-y

    pMap = new unsigned char[mapParams.width*mapParams.height]; // 900 * 900 的地图

    // cnt法
    Misses_cnt = new unsigned int[mapParams.width*mapParams.height];
    Hits_cnt = new unsigned int[mapParams.width*mapParams.height];

    // tsdf法
    pMapW = new unsigned int[mapParams.width*mapParams.height];
    pMapTSDF = new double[mapParams.width*mapParams.height];

   // 初始化为50  状态未知
   for(int i = 0; i < mapParams.width * mapParams.height;i++) {

      pMap[i] = 50;
      Misses_cnt[i] = 0;   
      Hits_cnt[i] = 0;
      pMapW[i] = 0;
      pMapTSDF[i] = -1;
   }
        
}


// input： 地图坐标系下的坐标
// return： 像素坐标系的坐标
// xy与origin都是map下的坐标
// 像素坐标系 = 栅格坐标系
GridIndex ConvertWorld2GridIndex(double x,double y)
{
    GridIndex index;
    // std::ceil   -->向上取整数   std::floor -->向下取整数
    index.x = std::ceil((x - mapParams.origin_x) / mapParams.resolution) + mapParams.offset_x;  // 设置了一个像素坐标的便宜值
    index.y = std::ceil((y - mapParams.origin_y) / mapParams.resolution) + mapParams.offset_y;
    // index.x = std::ceil((x - mapParams.origin_x) / mapParams.resolution);
    // index.y = std::ceil((y - mapParams.origin_y) / mapParams.resolution);
    return index;
}
// 插值函数
double interpolation(double A, double B, double a, double b) {
    if (a==b)   return A;

    double value = (b*A - a*B) / (b-a);
    return value;

}
double GridIndex2ConvertWorld(int x) {
  double temp_x;
  temp_x = (x - mapParams.offset_x)*mapParams.resolution + mapParams.origin_x;

  return temp_x;
}

// 地图大小固定之后，地图的更新，从地图左下角开始更新，从左到右，从下往上
int GridIndexToLinearIndex(GridIndex index)  
{
    int linear_index;
    linear_index = index.y + index.x * mapParams.width;
}


//判断index是否有效
bool isValidGridIndex(GridIndex index)
{
    if(index.x >= 0 && index.x < mapParams.width && index.y >= 0 && index.y < mapParams.height)
        return true;

    return false;
}

void DestoryMap()
{
    if(pMap != NULL)
        delete pMap;
}

// 激光数据
// map下，机器人位值（x，y，theta）
void OccupanyMapping(std::vector<GeneralLaserScan>& scans,std::vector<Eigen::Vector3d>& robot_poses)
{
    //枚举所有的激光雷达数据
    for(int i = 0; i < scans.size();i++)
    {
        GeneralLaserScan scan = scans[i]; // 深度+角度
        Eigen::Vector3d robotPose = robot_poses[i];// x y theta

        // 机器人坐标 对应的栅格坐标
        GridIndex robotIndex = ConvertWorld2GridIndex(robotPose(0),robotPose(1)); 

        for(int id = 0; id < scan.range_readings.size();id++) // 一帧
        {
            double dist = scan.range_readings[id];  // 激光深度
            double angle = scan.angle_readings[id]; // 在激光系下的偏角
            // 去除异常数据
            if(std::isinf(dist) || std::isnan(dist)) continue;

            // 2d平面的激光点， 激光点在激光系下的坐标
            double theta = robotPose(2);    // 机器人在地图下的偏角
            double laser_x = dist * cos(angle);  
            double laser_y = dist * sin(angle);
            // 激光点在地图下的坐标 2d空间的旋转矩阵
            double world_x = cos(theta) * laser_x - sin(theta) * laser_y + robotPose(0);
            double world_y = sin(theta) * laser_x + cos(theta) * laser_y + robotPose(1);
            // 激光点的栅格坐标
            GridIndex pointGridIndex = ConvertWorld2GridIndex(world_x, world_y);  
            //start of TODO 对对应的map的cell进行更新．
             OccGridMethod(pointGridIndex, robotIndex);     //1-占有法
            // CntGridMethod(pointGridIndex, robotIndex, 0.35);  //2-计数法
            // TSDFGridMethod(pointGridIndex, robotIndex, robotPose, dist);       //3-tsdf
           
        } 
    }
    //start of TODO
    // 2-计数法 统计每个栅格的占有率
    // for (int i=0; i< mapParams.height*mapParams.width; ++i) {
    //   if ((Misses_cnt[i] + Hits_cnt[i])!=0) {   // 未扫到栅格=50 未知
    //     double r = double(Hits_cnt[i] / (Misses_cnt[i] + Hits_cnt[i]));
    //     if (r >= 0.35)    // 占有
    //       pMap[i] = 100;
    //     else
    //       pMap[i] = r*100; // 空闲
    //   }
    // }
    //end of TODO

    //start of TODO
    // 3-tsdf
    /*for(int i= 0; i<mapParams.height-2; i++)  //假设i为y轴
    {
        for(int j = 0; j<mapParams.width-2; j++)  //假设j为x轴  
        { 
            int line_value = j + i*mapParams.height;
            int line_x = line_value +1;               //往x轴移动一格   右侧
            int line_y = line_value +mapParams.height;//往y轴方向移动一格   下侧
            //转map坐标
            double A_x = GridIndex2ConvertWorld(j);
            double A_y = GridIndex2ConvertWorld(i);
            double B_x = GridIndex2ConvertWorld(j+1);
            double B_y = GridIndex2ConvertWorld(i+1);
            double a,b,b1,x,y;
            a = pMapTSDF[line_value];  b= pMapTSDF[line_x];  b1 = pMapTSDF[line_y]; // 判断该点的下边、右边点是否为交界处

            if( a*b1 < 0){ //x方向   乘积为负数，交界处
                   x = A_x;
                   y = interpolation(A_y,B_y,a,b1);     //插值
                  pMap[GridIndexToLinearIndex(ConvertWorld2GridIndex(x,y))] = 100;
            }
            else if( a*b < 0){ //y方向
                   x = interpolation(A_x,B_x,a,b);
                   y = A_y;
                   pMap[GridIndexToLinearIndex(ConvertWorld2GridIndex(x,y))] = 100;
            }
                   
        }
    }

	*/

    //end of TODO
}


//发布地图．
void PublishMap(ros::Publisher& map_pub)
{
    nav_msgs::OccupancyGrid rosMap;

    // reset param
    rosMap.info.resolution = mapParams.resolution;  //分辨率
    rosMap.info.width = mapParams.width;    // 长宽
    rosMap.info.height = mapParams.height;
    rosMap.info.origin.position.x = 0.0;    // 原点位置
    rosMap.info.origin.position.y = 0.0;
    rosMap.info.origin.position.z = 0.0;
    rosMap.info.origin.orientation.x = 0.0;
    rosMap.info.origin.orientation.y = 0.0;
    rosMap.info.origin.orientation.z = 0.0;
    rosMap.info.origin.orientation.w = 1.0;
    rosMap.data.resize(rosMap.info.width * rosMap.info.height); // 栅格值


    // load data
    //0~100
    int cnt0,cnt1,cnt2;
    cnt0 = cnt1 = cnt2 = 100; 
    for(int i = 0; i < mapParams.width * mapParams.height;i++)
    {
       if(pMap[i] == 50)    // 未知
       {
           rosMap.data[i] = -1.0;
       }
       else
       {
           rosMap.data[i] = pMap[i];    // 已知
       }
    }
    rosMap.info.origin.position.x = mapParams.origin_x;
    rosMap.info.origin.position.y = mapParams.origin_y;
    rosMap.header.stamp = ros::Time::now();
    rosMap.header.frame_id = "map";

    map_pub.publish(rosMap);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "OccupanyMapping");

    ros::NodeHandle nodeHandler;

    ros::Publisher mapPub = nodeHandler.advertise<nav_msgs::OccupancyGrid>("laser_map",1,true);

    std::vector<Eigen::Vector3d> robotPoses;    // 机器人的位置
    std::vector<GeneralLaserScan> generalLaserScans;  // 激光数据

    std::string basePath = "/home/gj/Desktop/2d_map/src/occupany_mapping/data";

    std::string posePath= basePath + "/pose.txt";
    std::string anglePath = basePath + "/scanAngles.txt";
    std::string scanPath = basePath + "/ranges.txt";

    //读取数据
    ReadPoseInformation(posePath, robotPoses);  
    ReadLaserScanInformation(anglePath, scanPath, generalLaserScans); 

    //设置地图信息
    SetMapParams();
    OccupanyMapping(generalLaserScans,robotPoses);
    PublishMap(mapPub);

    ros::spin();

    DestoryMap();

    std::cout <<"Release Memory!!"<<std::endl;

}

// 激光点对应的栅格坐标
// 机器人对应的栅格坐标
// 占据栅格法
void OccGridMethod(GridIndex& pointGridIndex, GridIndex& robotIndex) { 

//如果二维平面，该机器人可以获得该激光点，那么机器人与激光点连线上肯定是空闲的
    std::vector<GridIndex> freeTraceindexes = TraceLine(robotIndex.x, robotIndex.y, pointGridIndex.x, pointGridIndex.y);
    for(auto& index : freeTraceindexes) // 遍历机器人位置到激光点之间的栅格(不包含激光点所在栅格)
    {
        if(!isValidGridIndex(index))
          continue;
        
        int linearIndex = GridIndexToLinearIndex(index); // 取出该栅格对应的值
        // 1-根据空闲规则  初始值默认50
        if(pMap[linearIndex] == 0)  
            continue;
        pMap[linearIndex] += mapParams.log_free;    // mapParams.log_free = -1 
        
    }
    // 激光点所在栅格肯定是障碍物，机器人所走过的路径都是空闲的
    if(isValidGridIndex(pointGridIndex))    // 激光点对应的栅格
    {
        int linearIndex = GridIndexToLinearIndex(pointGridIndex);
        // 2-根据占据规则 
        pMap[linearIndex] += mapParams.log_occ;     // mapParams.log_occ = 2
        if(pMap[linearIndex] > 100)   
            pMap[linearIndex] = 100;
    }
}

// 计数栅格法
// input: 激光点的像素坐标、机器人的像素坐标、占据率
void CntGridMethod(GridIndex& pointGridIndex, GridIndex& robotIndex, double occRate) { 

    std::vector<GridIndex> freeTraceindexes = TraceLine(robotIndex.x, robotIndex.y, pointGridIndex.x, pointGridIndex.y);
    for(auto& index : freeTraceindexes) // 遍历机器人位置到激光点之间的栅格(不包含激光点所在栅格),机器人所走过的路径都是空闲的
    {
        if(!isValidGridIndex(index))
          continue;
        
        int linearIndex = GridIndexToLinearIndex(index); // 取出该栅格对应的值
        // 根据空闲规则
        Misses_cnt[linearIndex]++;
        
    }
    // 激光点所在栅格肯定是障碍物
    if(isValidGridIndex(pointGridIndex))    // 激光点对应的栅格
    {
        int linearIndex = GridIndexToLinearIndex(pointGridIndex);
        // 根据占据规则 
        Hits_cnt[linearIndex]++;
    }
    // // 统计每个栅格的占有率  最后计算完，再统一算占有率
    // for (int i=0; i< mapParams.height*mapParams.width; ++i) {

    //   if ((Misses_cnt[i] + Hits_cnt[i])!=0) {   // 未扫到栅格=50 未知
    //     double r = double(Hits_cnt[i] / (Misses_cnt[i] + Hits_cnt[i]));
    //     if (r >= occRate)
    //       pMap[i] = 100;   // 占有
    //     else
    //       pMap[i] = r*100; 
    //   }
    // }
}


// tsdf栅格法：阶段式带符号距离函数（加权最小线性二乘）
// 激光点栅格坐标、机器人地图坐标、当前帧激光的深度
void TSDFGridMethod(GridIndex& pointGridIndex, GridIndex& robotIndex, Eigen::Vector3d& robotPose, double& dist) { 

  //111
    std::vector<GridIndex> freeTraceindexes = TraceLine(robotIndex.x, robotIndex.y, pointGridIndex.x, pointGridIndex.y);
    for(auto& index : freeTraceindexes) // 遍历机器人位置到激光点之间的栅格(不包含激光点所在栅格),机器人所走过的路径都是空闲的
    {
        if(!isValidGridIndex(index))
          continue;
        // 栅格坐标系转换到map下
        double x = (index.x - mapParams.offset_x) * mapParams.resolution + mapParams.origin_x;
        double y = (index.y - mapParams.offset_y) * mapParams.resolution + mapParams.origin_y;
        // 计算map下，激光点所在栅格与机器人位置的距离
        // 激光距离-计算的距离
        double d = std::sqrt( pow((x-robotPose(0)), 2) + pow((y-robotPose(1)), 2) );  // 栅格里传感器原点的距离
        double sdf = dist-d;
        double t = 0.1;   // 截止距离
        double tsdf = std::max(-1.0, std::min(1.0, sdf/t));    // tsdfi(x)，范围-1 to 1

        int linearIndex = GridIndexToLinearIndex(index);
        // 测量更新
        // 第一次 TSDFi = tsdfi，Wi = 1
        // 小w=1
        pMapTSDF[linearIndex] = ( pMapW[linearIndex]*pMapTSDF[linearIndex] + 1.0*tsdf ) / (pMapW[linearIndex] + 1); // 更新TSDFi(x)
        pMapW[linearIndex] += 1;         // 更新Wi(x)

    }
  //222
    // 遍历tsdf场，找正负交界处，插值为0的栅格即为占用栅格
    // for(int i= 0; i<mapParams.height-2; i++)  //假设i为y轴
    // {
    //     for(int j = 0; j<mapParams.width-2; j++)  //假设j为x轴  
    //     { 
    //         int line_value = j + i*mapParams.height;
    //         int line_x = line_value +1;               //往x轴移动一格   右侧
    //         int line_y = line_value +mapParams.height;//往y轴方向移动一格   下侧
    //         //转map坐标
    //         double A_x = GridIndex2ConvertWorld(j);
    //         double A_y = GridIndex2ConvertWorld(i);
    //         double B_x = GridIndex2ConvertWorld(j+1);
    //         double B_y = GridIndex2ConvertWorld(i+1);
    //         double a,b,b1,x,y;
    //         a = pMapTSDF[line_value];  b= pMapTSDF[line_x];  b1 = pMapTSDF[line_y]; // 判断该点的下边、右边点是否为交界处

    //         if( a*b1 < 0){ //x方向   乘积为负数，交界处
    //                x = A_x;
    //                y = interpolation(A_y,B_y,a,b1);     //插值
    //               pMap[GridIndexToLinearIndex(ConvertWorld2GridIndex(x,y))] = 100; 
    //         }
    //         else if( a*b < 0){ //y方向
    //                x = interpolation(A_x,B_x,a,b);
    //                y = A_y;
    //                pMap[GridIndexToLinearIndex(ConvertWorld2GridIndex(x,y))] = 100;
    //         }
                   
    //     }
    // }

}


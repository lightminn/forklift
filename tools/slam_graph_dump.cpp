// Read a saved slam_toolbox pose graph (<prefix>.posegraph + <prefix>.data, boost binary archives) and write what
// the SLAM localization plan needs from it (docs/plans/2026-10-20-slam-localization-session.md L2, V2'):
//   <out>/nodes.csv   one line per processed scan: unique id, state id, stamp, corrected and odometric base pose
//   <out>/graph.json  vertex and edge counts, the occupancy grid's size, origin and resolution
//   <out>/edges.csv   one line per graph edge: the source and target scans' unique ids (slam-bounded-graph plan 4.3:
//                     components and reachability after node removal)
//   <out>/grid.u8     that grid, karto's GridStates (0 unknown, 100 occupied, 255 free), row-major from the origin
// The grid is the one slam_toolbox publishes on /map for this graph (SMapper::getOccupancyGrid: CreateFromScans over
// every processed scan with the given resolution, min_pass_through and occupancy_threshold).
// Built and run inside forklift/ros2-dev:jazzy by tools/slam_bundle.sh.
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "karto_sdk/Mapper.h"

static int dump(int argc, char ** argv);

int main(int argc, char ** argv)
{
  try {
    return dump(argc, argv);
  } catch (const karto::Exception & e) {
    std::cerr << "karto exception: " << e.GetErrorMessage() << "\n";
    return 3;
  }
}

static int dump(int argc, char ** argv)
{
  if (argc != 6) {
    std::cerr << "usage: slam_graph_dump <bundle prefix> <out dir> <resolution> <min_pass_through> "
      "<occupancy_threshold>\n";
    return 2;
  }
  const std::string prefix = argv[1], out = argv[2];
  const double resolution = std::stod(argv[3]);
  const int min_pass = std::stoi(argv[4]);
  const double threshold = std::stod(argv[5]);

  karto::Mapper * mapper = new karto::Mapper();
  karto::Dataset * dataset = new karto::Dataset();
  mapper->LoadFromFile(prefix + ".posegraph");
  dataset->LoadFromFile(prefix + ".data");
  // A scan finds its laser by name in the sensor registry, as after slam_toolbox's own deserialisation.
  for (karto::Object * object : dataset->GetLasers()) {
    karto::Sensor * laser = dynamic_cast<karto::Sensor *>(object);
    if (laser == nullptr) {
      continue;
    }
    try {
      karto::SensorManager::GetInstance()->GetSensorByName(laser->GetName());  // throws when not registered
    } catch (const karto::Exception &) {
      karto::SensorManager::GetInstance()->RegisterSensor(laser);
    }
  }

  const karto::LocalizedRangeScanVector scans = mapper->GetAllProcessedScans();
  std::ofstream nodes(out + "/nodes.csv");
  nodes << "unique_id,state_id,stamp_s,x,y,yaw,odom_x,odom_y,odom_yaw\n" << std::setprecision(17);
  for (karto::LocalizedRangeScan * scan : scans) {
    const karto::Pose2 & c = scan->GetCorrectedPose();
    const karto::Pose2 & o = scan->GetOdometricPose();
    nodes << scan->GetUniqueId() << ',' << scan->GetStateId() << ',' << scan->GetTime() << ',' << c.GetX() << ','
          << c.GetY() << ',' << c.GetHeading() << ',' << o.GetX() << ',' << o.GetY() << ',' << o.GetHeading() << '\n';
  }

  size_t vertices = 0;
  for (const auto & named : mapper->GetGraph()->GetVertices()) {
    vertices += named.second.size();
  }
  const size_t edges = mapper->GetGraph()->GetEdges().size();
  std::ofstream edge_rows(out + "/edges.csv");
  edge_rows << "source,target\n";
  for (karto::Edge<karto::LocalizedRangeScan> * edge : mapper->GetGraph()->GetEdges()) {
    edge_rows << edge->GetSource()->GetObject()->GetUniqueId() << ','
              << edge->GetTarget()->GetObject()->GetUniqueId() << '\n';
  }

  karto::OccupancyGrid * grid = karto::OccupancyGrid::CreateFromScans(scans, resolution, min_pass, threshold);
  std::ofstream meta(out + "/graph.json");
  meta << std::setprecision(17) << "{\"scans\": " << scans.size() << ", \"vertices\": " << vertices
       << ", \"edges\": " << edges;
  if (grid != nullptr) {
    const karto::Vector2<kt_double> offset = grid->GetCoordinateConverter()->GetOffset();
    meta << ", \"width\": " << grid->GetWidth() << ", \"height\": " << grid->GetHeight() << ", \"origin\": ["
         << offset.GetX() << ", " << offset.GetY() << "], \"resolution_m\": " << grid->GetResolution();
    std::ofstream cells(out + "/grid.u8", std::ios::binary);
    for (kt_int32s y = 0; y < grid->GetHeight(); y++) {
      for (kt_int32s x = 0; x < grid->GetWidth(); x++) {
        const kt_int8u value = grid->GetValue(karto::Vector2<kt_int32s>(x, y));
        cells.put(static_cast<char>(value));
      }
    }
  }
  meta << "}\n";
  return grid != nullptr && nodes.good() && meta.good() && edge_rows.good() ? 0 : 1;
}
